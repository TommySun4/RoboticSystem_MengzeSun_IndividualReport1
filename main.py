"""MotorLab (StampC3) report figures from the actual recorded CSV files.

Run in PyCharm (working directory is irrelevant):
    python motor_report_figures.py --data data --out results/report_figures

Inputs may be CSV or a ZIP containing one CSV. Handles # comments and repeated headers.
Left wheel positive direction; raw files are not modified.
"""
import argparse
from pathlib import Path
import zipfile

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

CPR = 358.3
PHASE = "stage"
SOURCES = {
    "nonlinear": ["L_nonlinear.csv", "L_nonlinear(1).csv"],
    "gain": ["L_gain.csv", "L_gain(1).csv"],
    "saturation": ["L_saturation_high.csv", "L_saturation(2).csv", "L_saturation(1).csv", "L_saturation.csv"],
    "final_deadband": ["L_deadband_final.csv", "L_deadband_final.csv.zip"],
    "step": ["L_step.csv", "L_step(1).csv"],
}

plt.rcParams.update({
    "font.family": "DejaVu Sans", "font.size": 9,
    "axes.titlesize": 11, "axes.labelsize": 10,
    "savefig.dpi": 300, "pdf.fonttype": 42,
})


def find_input(folder, names):
    for name in names:
        p = folder / name
        if p.is_file():
            return p
    return None


def read_motorlab(path):
    if path.suffix.lower() == ".zip":
        with zipfile.ZipFile(path) as z:
            names = [s for s in z.namelist()
                     if s.lower().endswith(".csv") and not s.startswith("__MACOSX/")]
            if len(names) != 1:
                raise ValueError(f"Expected one CSV in {path.name}; found {names}")
            with z.open(names[0]) as f:
                df = pd.read_csv(f, comment="#", low_memory=False)
    else:
        df = pd.read_csv(path, comment="#", low_memory=False)

    df.columns = df.columns.str.strip().str.replace("\ufeff", "", regex=False)
    needed = ["run", "trial", "stage", "ms", "dt", "Lpwm", "Lcount", "Lrpm"]
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise ValueError(f"Unexpected CSV columns in {path.name}: missing {missing}")
    # The controller sometimes repeats the header halfway through the CSV.
    for c in ["run", "trial", "ms", "dt", "Lpwm", "Lcount", "Lrpm", "cmd_ms"]:
        if c in df:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    df["stage"] = df["stage"].astype(str).str.strip().str.upper()
    df = df.dropna(subset=["run", "trial", "ms", "dt", "Lpwm", "Lcount", "Lrpm"])
    df = df[df["dt"] > 0].copy()
    df[["run", "trial"]] = df[["run", "trial"]].astype(int)
    return df.sort_values(["run", "trial", "ms"]).reset_index(drop=True)


def per_trial_speed(df):
    """One speed observation per MEASURE trial, not one per 10ms sample."""
    rows = []
    selected = df[(df[PHASE] == "MEASURE") & (df["Lpwm"] >= 0)]
    for (run, trial), g in selected.groupby(["run", "trial"]):
        if len(g) < 30 or g["ms"].max() - g["ms"].min() < 300:
            continue  # exclude incomplete measurement windows
        if g["Lpwm"].nunique() != 1:
            continue
        # Time-weighted average reduces the influence of quantised RPM spikes.
        rpm = abs(np.average(g["Lrpm"], weights=g["dt"]))
        rows.append(dict(run=run, trial=trial, pwm=float(g["Lpwm"].iloc[0]),
                         rpm=float(rpm), samples=len(g)))
    return pd.DataFrame(rows, columns=["run", "trial", "pwm", "rpm", "samples"])


def mean_by_pwm(trials):
    if trials.empty:
        return pd.DataFrame(columns=["pwm", "mean", "sd", "n"])
    out = trials.groupby("pwm", as_index=False).agg(
        mean=("rpm", "mean"), sd=("rpm", "std"), n=("rpm", "size"))
    out["sd"] = out["sd"].fillna(0.0)
    return out.sort_values("pwm")


def save(fig, out, name):
    fig.tight_layout()
    for ext in ("pdf", "png"):
        fig.savefig(out / f"{name}.{ext}", bbox_inches="tight", dpi=300)
    plt.close(fig)


def motor_gain(gain):
    fit = gain[(gain["pwm"] >= 30) & (gain["pwm"] <= 60)]
    if len(fit) < 3:
        return None
    x = fit["pwm"].to_numpy(dtype=float)
    y = fit["mean"].to_numpy(dtype=float)
    slope, intercept = np.polyfit(x, y, 1)
    predicted = slope * x + intercept
    r2 = 1 - np.sum((y - predicted)**2) / np.sum((y - y.mean())**2)
    return slope, intercept, r2


def full_pwm_figure(stats, gain_fit, out):
    fig, ax = plt.subplots(figsize=(8, 4.7))
    labels = {
        "nonlinear": "Nonlinearity: PWM 0–60",
        "gain": "Gain: PWM 30–60",
        "saturation": "High-PWM: PWM 60–100",
    }
    for name in ("nonlinear", "saturation", "gain"):
        s = stats.get(name)
        if s is None or s.empty:
            continue
        ax.errorbar(s["pwm"], s["mean"], yerr=s["sd"], fmt="o-" if name != "gain" else "s",
                    capsize=2.5, linewidth=1.6, ms=4.3, label=labels[name], alpha=0.88)
    ax.axvspan(23, 29, color="grey", alpha=.10, label="Intermittent startup zone (23–29)")
    ax.set(xlabel="Applied PWM command", ylabel="Settled left-wheel speed |RPM|",
           title="Left Motor — PWM vs Wheel Speed")
    ax.set_xlim(-3, 105)
    ax.set_ylim(bottom=-5)
    ax.grid(alpha=.2)
    ax.legend(loc="upper left", fontsize=7.5, framealpha=.9)
    notes = ["Wheel L; positive direction", "Steady-state samples: 500 ms", "Error bars: between-trial ±1 SD"]
    if gain_fit is not None:
        notes.append(f"Gain (PWM 30–60): {gain_fit[0]:.3f} RPM/PWM")
    notes.append("No clear high-PWM plateau observed ≤ 100")
    ax.text(.98, .04, "\n".join(notes), transform=ax.transAxes, ha="right", va="bottom", fontsize=7.3,
            bbox=dict(boxstyle="round,pad=.4", fc="white", ec="0.7", alpha=.94))
    save(fig, out, "01_full_pwm_rpm")


def gain_figure(gain, fit, out):
    if gain is None or gain.empty or fit is None:
        return
    a, b, r2 = fit
    fig, ax = plt.subplots(figsize=(7.2, 4.1))
    ax.errorbar(gain["pwm"], gain["mean"], yerr=gain["sd"], fmt="o", capsize=4,
                label="Measured mean ± between-trial SD", ms=5)
    u = np.linspace(30, 60, 250)
    ax.plot(u, a*u+b, "--", linewidth=1.7, label="Linear least-squares fit")
    ax.set(xlabel="PWM command", ylabel="Settled left-wheel speed |RPM|",
           title="Left Motor — Gain in the Linear Region")
    ax.grid(alpha=.23)
    ax.legend(loc="upper left", fontsize=8)
    ax.text(.98, .06, f"PWM range: 30–60\nRepeats: {int(gain['n'].min())} per level"
            f"\nRPM = {a:.5f} × PWM {b:+.4f}\nGain G = {a:.4f} RPM/PWM\nR² = {r2:.5f}",
            transform=ax.transAxes, ha="right", va="bottom", fontsize=8,
            bbox=dict(boxstyle="round,pad=.5", fc="white", ec="0.75", alpha=.95))
    save(fig, out, "03_gain_fit")


def wrapped_difference(new, old):
    """Signed 32-bit encoder-counter difference, robust at rollover."""
    delta = int(new) - int(old)
    if delta > 2**31:
        delta -= 2**32
    elif delta < -(2**31):
        delta += 2**32
    return delta


def trial_sustained(g, min_counts=1, consecutive=3):
    """Match MotorLab's criterion: SETTLE+MEASURE, ≥100ms bins, 3 in a row."""
    g = g[g["stage"].isin(["SETTLE", "MEASURE"])].sort_values("ms")
    if len(g) < 60 or not {"SETTLE", "MEASURE"}.issubset(set(g["stage"])):
        return None
    durations, steps, bins = 0, 0, []
    last_count = None
    for row in g.itertuples(index=False):
        current = row.Lcount
        if last_count is not None:
            steps += wrapped_difference(current, last_count)
            durations += int(row.dt)
            if durations >= 100:
                bins.append(abs(steps))
                steps, durations = 0, 0
        last_count = current
    if len(bins) < consecutive:
        return None
    streak = longest = 0
    for count in bins:
        streak = streak + 1 if count >= min_counts else 0
        longest = max(longest, streak)
    return int(longest >= consecutive)


def startup_table(df):
    rows = []
    for (run, trial), g in df.groupby(["run", "trial"]):
        p = g.loc[g["stage"] == "MEASURE", "Lpwm"]
        if p.empty or p.nunique() != 1:
            continue
        success = trial_sustained(g)
        if success is not None:
            rows.append({"run":run, "trial":trial, "pwm":float(p.iloc[0]),
                         "sustained":success})
    return pd.DataFrame(rows, columns=["run", "trial", "pwm", "sustained"])


def wilson_bounds(k, n):
    z = 1.95996398454
    p = k/n
    den = 1 + z*z/n
    center = (p + z*z/(2*n))/den
    half = z * np.sqrt(p*(1-p)/n + z*z/(4*n*n))/den
    return max(0, center-half), min(1, center+half)


def startup_figure(trials, out):
    if trials.empty:
        return
    tab = trials.groupby("pwm", as_index=False).agg(n=("sustained", "size"),
                                                      success=("sustained", "sum"))
    tab["percent"] = 100 * tab["success"]/tab["n"]
    tab.to_csv(out / "deadband_success_rates.csv", index=False)
    bounds = [wilson_bounds(int(r.success), int(r.n)) for r in tab.itertuples()]
    lo = np.array([v[0] for v in bounds])*100
    hi = np.array([v[1] for v in bounds])*100
    values = tab["percent"].to_numpy()
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    ax.bar(tab["pwm"], values, width=.65, alpha=.78, label="Sustained starts / 5")
    ax.errorbar(tab["pwm"], values, yerr=np.vstack([values-lo, hi-values]),
                fmt="none", capsize=4, color="black", elinewidth=1,
                label="Wilson 95% confidence interval")
    for r in tab.itertuples():
        ax.annotate(f"{int(r.success)}/{int(r.n)}", xy=(r.pwm, r.percent),
                    xytext=(0, 8), textcoords="offset points", ha="center", fontsize=8)
    reliable = tab[(tab["n"] >= 3) & (tab["success"] == tab["n"])]
    first_label = f"; first full success: PWM {reliable['pwm'].min():g}" if not reliable.empty else ""
    ax.set(ylim=(0, 125), xlabel="PWM command", ylabel="Sustained-start success rate (%)",
           title=f"Left Motor — Startup Reliability (n=5 per PWM{first_label})")
    ax.set_xticks(tab["pwm"])
    ax.grid(axis="y", alpha=.2)
    ax.legend(loc="upper right", fontsize=7.5)
    ax.text(.02, .04, "Criterion: ≥1 net encoder count per ≥100 ms bin,\n"
            "for ≥3 consecutive bins (SETTLE + MEASURE).\n"
            "Confidence intervals reflect only five trials per PWM.",
            transform=ax.transAxes, ha="left", va="bottom", fontsize=7.3,
            bbox=dict(boxstyle="round,pad=.3", fc="white", ec="0.8", alpha=.95))
    save(fig, out, "02_startup_reliability")


def step_figure(df, out):
    fig, ax = plt.subplots(figsize=(7.2, 4.1))
    seen = 0
    for (run, trial), g in df.groupby(["run", "trial"]):
        q = g[g["stage"].isin(["INITIAL", "FINAL"])].sort_values("ms").copy()
        final = q[q["stage"] == "FINAL"]
        if len(q) < 70 or final.empty:
            continue
        # cmd_ms records the acknowledged PWM transition timestamp.
        t0 = final["cmd_ms"].iloc[0] if "cmd_ms" in final and pd.notna(final["cmd_ms"].iloc[0]) else final["ms"].iloc[0]
        q["t"] = (q["ms"]-t0)/1000
        smoothed = q["Lrpm"].rolling(5, center=True, min_periods=1).mean()
        ax.plot(q["t"], smoothed, label=f"Trial {int(trial)}", lw=1.4, alpha=.85)
        seen += 1
    if seen == 0:
        plt.close(fig)
        return
    ax.axvline(0, linestyle="--", color="black", linewidth=1, label="PWM 30→60")
    ax.set(xlabel="Time from PWM step (s)", ylabel="Left wheel speed (RPM)",
           title="Left Motor — Step Response")
    ax.grid(alpha=.2)
    ax.legend(loc="upper left", fontsize=8)
    ax.text(.98, .05, f"Input: PWM 30 → 60\nInitial: 700 ms; final: 700 ms\nTrials: {seen}\n50 ms rolling mean (visualisation only)",
            transform=ax.transAxes, ha="right", va="bottom", fontsize=7.5,
            bbox=dict(boxstyle="round,pad=.4", fc="white", ec="0.75", alpha=.95))
    save(fig, out, "04_step_response")


def main():
    parser = argparse.ArgumentParser(description="Create labelled report graphs from MotorLab CSV data")
    base = Path(__file__).resolve().parent
    parser.add_argument("--data", type=Path, default=base / "data")
    parser.add_argument("--out", type=Path, default=base / "results" / "report_figures")
    args = parser.parse_args()
    folder = args.data.resolve()
    out = args.out.resolve()
    out.mkdir(parents=True, exist_ok=True)
    if not folder.exists():
        raise SystemExit(f"Data folder does not exist: {folder}")
    stats, gain_fit = {}, None
    for name in ("nonlinear", "gain", "saturation"):
        source = find_input(folder, SOURCES[name])
        if source is None:
            print(f"[SKIP] {name}: CSV missing, accepted names: {SOURCES[name]}")
            continue
        trials = per_trial_speed(read_motorlab(source))
        summary = mean_by_pwm(trials)
        summary.to_csv(out / f"{name}_summary.csv", index=False)
        stats[name] = summary
        print(f"[OK] {name}: {len(trials)} trials from {source.name}")
    if "gain" in stats:
        gain_fit = motor_gain(stats["gain"])
        if gain_fit:
            print(f"Gain fit: RPM = {gain_fit[0]:.5f}*PWM {gain_fit[1]:+.5f}; R²={gain_fit[2]:.5f}")
    if stats:
        full_pwm_figure(stats, gain_fit, out)
        gain_figure(stats.get("gain"), gain_fit, out)
    source = find_input(folder, SOURCES["final_deadband"])
    if source is not None:
        deadband = startup_table(read_motorlab(source))
        if not deadband.empty:
            deadband.to_csv(out / "deadband_trials.csv", index=False)
            startup_figure(deadband, out)
            print("[OK] deadband final:", len(deadband), "trials")
        else:
            print("[WARN] no complete deadband trials")
    else:
        print("[SKIP] final deadband: put L_deadband_final.csv[.zip] into data/")
    source = find_input(folder, SOURCES["step"])
    if source is not None:
        step_figure(read_motorlab(source), out)
        print("[OK] step response")
    print(f"\nFinished! Open this folder:\n{out}")


if __name__ == "__main__":
    main()
