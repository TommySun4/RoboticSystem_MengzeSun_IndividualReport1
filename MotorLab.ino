#include <Arduino.h>
#include <stdlib.h>
#include <string.h>
#include "Config.h"
#include "RobotBus.h"
#include "EncoderSpeed.h"
#include "Experiment.h"

RobotBus bus;
Experiment experiment;
EncoderSpeed leftSpeed, rightSpeed;
bool busReady = false, fault = true, teamReady = false;
bool requireTeam = Config::REQUIRE_TEAM, bootPending = true;
uint32_t bootAt = 0, bootAttemptAt = 0, lastPlot = 0;
uint32_t outputIntervalMs = 200; // Plot/text output throttling; CSV keeps full sampling resolution.
uint32_t lastSample = 0, lastZero = 0, lastIdleLog = 0, sampleNumber = 0;
uint32_t previousL = 0, previousR = 0, previousTime = 0, appliedAt = 0;
int appliedL = 0, appliedR = 0;
bool haveSample = false;
float rpmL = 0, rpmR = 0;
uint32_t dropped = 0;
const char *reason = "BOOT";
char outputMode = 't'; // text, plot, csv
char criticalLine[224] = {};
unsigned criticalOffset = 0;
int sentPhase = -1;
uint32_t sentTrial = 0;
const char *modeName() {
  const char *names[]={"start","ramp","sequence","manual","speed","deadband","saturation","gain","nonlinear","step","continued"};
  return names[int(experiment.cfg.mode)];
}
const char *stageName() {
  const char *names[]={"IDLE","REST","RUN","SETTLE","MEASURE","INITIAL","FINAL","COOL","DONE","ABORT"};
  return names[int(experiment.phase)];
}

// Small bounded output queue. Flushes only into available TX capacity.
constexpr unsigned QUEUE_SIZE = 128, ROW_SIZE = 224;
char rows[QUEUE_SIZE][ROW_SIZE];
unsigned head = 0, tail = 0, rowOffset = 0;
bool push(const char *s) {
  unsigned next = (head + 1) % QUEUE_SIZE;
  if (next == tail) { ++dropped; return false; }
  snprintf(rows[head], ROW_SIZE, "%s", s); head = next; return true;
}
void flushOutput() {
  if (tail == head) {
    if (!criticalLine[0]) return;
    int available = Serial.availableForWrite();
    if (available <= 0) return;
    size_t remain=strlen(criticalLine)-criticalOffset;
    size_t amount=remain<size_t(available)?remain:size_t(available);
    criticalOffset+=Serial.write(reinterpret_cast<const uint8_t*>(criticalLine+criticalOffset),amount);
    if(criticalOffset==strlen(criticalLine)) {criticalLine[0]=0;criticalOffset=0;}
    return;
  }
  size_t remain = strlen(rows[tail]) - rowOffset;
  int available = Serial.availableForWrite();
  if (available <= 0) return;
  size_t amount = remain < size_t(available) ? remain : size_t(available);
  size_t sent = Serial.write(reinterpret_cast<const uint8_t*>(rows[tail] + rowOffset), amount);
  rowOffset += sent;
  if (rowOffset == strlen(rows[tail])) { tail = (tail + 1) % QUEUE_SIZE; rowOffset = 0; }
}
void header() {
  push("run,trial,stage,ms,dt,Lpwm,Rpwm,Lcount,Rcount,Lrpm,Rrpm,cmd_ms\n");
}
void reportPhase() {
  char text[160];
  if(sentTrial!=experiment.trial) {
    sentTrial=experiment.trial;
    snprintf(text,sizeof text,"# TRIAL run=%lu trial=%lu repeat=%d\n",
      (unsigned long)experiment.batch,(unsigned long)experiment.trial,experiment.repeat);
    push(text);
  }
  if(sentPhase!=int(experiment.phase)) {
    sentPhase=int(experiment.phase);
    snprintf(text,sizeof text,"# PHASE run=%lu trial=%lu stage=%s\n",
      (unsigned long)experiment.batch,(unsigned long)experiment.trial,stageName());
    push(text);
  }
}
int64_t signedBits(uint32_t v) { return v <= 0x7fffffffU ? int64_t(v) : int64_t(v)-0x100000000LL; }
void halt(const char *why, bool failed) {
  bool wasActive=experiment.active;
  // Repeated stop while already idle is intentionally silent.
  if (!wasActive && !failed && !fault) {
    if (busReady) bus.motors(0,0);
    appliedL=appliedR=0;lastZero=millis();
    return;
  }
  bool partial=wasActive && experiment.phase!=Phase::Cool;
  bool repeated=fault&&!wasActive;
  const char *oldReason=reason;
  experiment.stop(failed||partial);
  if(failed) fault=true;
  reason=why;
  bool acknowledged=busReady && bus.motors(0,0);
  if(!acknowledged) {fault=true;reason="STOP_WRITE";}
  appliedL=appliedR=0;appliedAt=lastZero=millis();
  if(repeated && !strcmp(oldReason,reason)) return;
  char text[ROW_SIZE];
  snprintf(text,sizeof text,"# %s run=%lu trial=%lu reason=%s aborted=%d stop_ack=%d dropped=%lu\n",
    fault?"FAULT":"STOP",(unsigned long)experiment.batch,(unsigned long)experiment.trial,
    reason,int(partial||failed),int(acknowledged),(unsigned long)dropped);
  // If the queue is full, retain the critical event until queued rows drain.
  if(!push(text) && !criticalLine[0]) {
    snprintf(criticalLine,sizeof criticalLine,"# FAULT run=%lu trial=%lu reason=LOG_OVERFLOW dropped=%lu\n",
      (unsigned long)experiment.batch,(unsigned long)experiment.trial,(unsigned long)dropped);
    criticalOffset=0;
  }
  sentPhase=int(experiment.phase);
}
void startTrial() {
  if (bootPending || experiment.active || fault || (requireTeam && !teamReady) || !experiment.valid()) {
    push("# start rejected: check boot, fault, profile/team, ranges and timing; use status\n"); return;
  }
  if(criticalLine[0] || (tail+QUEUE_SIZE-head-1)%QUEUE_SIZE<8) {
    push("# WAIT output_pending; try again after logs drain\n");return;
  }
  uint32_t l, r;
  if (!bus.encoders(l,r)) { halt("START_READ",true); return; }
  uint32_t now = millis();
  previousL=l; previousR=r; previousTime=lastSample=now; haveSample=true;
  leftSpeed.reset(); rightSpeed.reset();
  leftSpeed.update(l,now,Config::COUNTS_PER_REV,Config::LEFT_ENCODER_SIGN);
  rightSpeed.update(r,now,Config::COUNTS_PER_REV,Config::RIGHT_ENCODER_SIGN);
  sampleNumber = 0;
  experiment.start(now);
  sentPhase=-1;sentTrial=0;
  char text[ROW_SIZE];
  snprintf(text,sizeof text,"# RUN run=%lu mode=%s wheel=%c dir=%d repeats=%d cpr=%.1f lsign=%d rsign=%d\n",
    (unsigned long)experiment.batch,modeName(),experiment.cfg.wheel,experiment.cfg.direction,
    experiment.cfg.repeats,double(Config::COUNTS_PER_REV),Config::LEFT_ENCODER_SIGN,Config::RIGHT_ENCODER_SIGN);
  push(text);
  snprintf(text,sizeof text,"# CONFIG rest=%lu settle=%lu measure=%lu cool=%lu u0=%d u1=%d initial=%lu final=%lu\n",
    (unsigned long)experiment.cfg.rest,(unsigned long)experiment.cfg.settle,(unsigned long)experiment.cfg.measure,
    (unsigned long)experiment.cfg.cool,experiment.cfg.u0,experiment.cfg.u1,
    (unsigned long)experiment.cfg.initial,(unsigned long)experiment.cfg.finalHold);push(text);
  if(outputMode=='c') header();
  reportPhase();
}
void status() {
  char text[ROW_SIZE];
  snprintf(text,sizeof text,"# STATUS state=%s mode=%s wheel=%c dir=%d pwm=%d cap=%d profile=%s team=%d format=%s\n",
    bootPending?"BOOT":fault?"FAULT":experiment.active?stageName():"READY",modeName(),experiment.cfg.wheel,
    experiment.cfg.direction,experiment.cfg.pwm,experiment.cfg.cap,requireTeam?"course":"example",int(teamReady),
    outputMode=='c'?"csv":outputMode=='p'?"plot":"text");push(text);
  if(experiment.cfg.mode>=Mode::Deadband) {
    snprintf(text,sizeof text,"# SETTINGS range=%d:%d:%d repeats=%d rest=%lu settle=%lu measure=%lu cool=%lu step=%d:%d:%lu:%lu\n",
      experiment.cfg.low,experiment.cfg.high,experiment.cfg.increment,experiment.cfg.repeats,
      (unsigned long)experiment.cfg.rest,(unsigned long)experiment.cfg.settle,(unsigned long)experiment.cfg.measure,
      (unsigned long)experiment.cfg.cool,experiment.cfg.u0,experiment.cfg.u1,
      (unsigned long)experiment.cfg.initial,(unsigned long)experiment.cfg.finalHold);push(text);
  }
  if(fault) {snprintf(text,sizeof text,"# CHECK reason=%s; inspect connection then recover\n",reason);push(text);}
  snprintf(text,sizeof text,"# OUTPUT interval_ms=%lu (plot/text only); idle=silent\n",(unsigned long)outputIntervalMs);push(text);
  if(experiment.cfg.mode==Mode::Manual) push("# NOTE manual = motors OFF; use mode speed for powered test\n");
}
bool number(const char *s, int &v) {
  if (!s || !s[0]) return false;
  char *end; long n = strtol(s,&end,10);
  if (*end || n < -60000 || n > 60000) return false;
  v=int(n); return true;
}
void command(char *line) {
  if (!strcmp(line,"s") || !strcmp(line,"stop")) { halt("USER",false); return; }
  if (!strcmp(line,"status")) { status(); return; }
  if (!strcmp(line,"help")) {
    push("# mode start|ramp|sequence|manual|speed|deadband|saturation|gain|nonlinear|step|continued\n");
    push("# wheel L|R|B; sign -1|1; pwm N; range LOW HIGH INC; repeats N; limit N\n");
    push("# times REST SETTLE MEASURE COOL; step U0 U1 INITIAL FINAL (times ms)\n");
    push("# interval MS (50..5000, plot/text only); quiet idle; ! immediate abort\n");
    push("# profile example|course; team NAME; format text|plot|csv; status; recover; start (or physical button); s\n"); return;
  }
  if (!strncmp(line,"profile ",8) && !experiment.active) {
    if (!strcmp(line+8,"course")) requireTeam=true;
    else if (!strcmp(line+8,"example")) requireTeam=false;
    else {push("# rejected: profile example|course\n");return;}
    status();return;
  }
  if (bootPending && strcmp(line,"help") && strcmp(line,"status")) {
    push("# boot handshake pending; wait for status boot_pending=0\n"); return;
  }
  if (experiment.active) { push("# busy: stop before changing configuration\n"); return; }
  if (!strcmp(line,"start")) { startTrial(); return; }
  if (!strcmp(line,"recover")) {
    uint32_t l,r;
    bool stopped=busReady && bus.motors(0,0);
    bool read=busReady && bus.encoders(l,r);
    fault=!(stopped&&read); reason=fault?"RECOVERY_FAILED":"READY";
    status(); return;
  }
  if (!strncmp(line,"team ",5)) {
    teamReady=busReady&&bus.team(line+5);
    if (!teamReady) halt("TEAM_WRITE_OR_NAME",true);
    else { push("# team configured (name omitted from log)\n"); status(); }
    return;
  }
  Settings candidate = experiment.cfg;
  char *tokens[6] = {}; unsigned count=0;
  for (char *part=strtok(line," "); part && count<6; part=strtok(nullptr," ")) tokens[count++]=part;
  if (!count) return;
  bool ok=false; int a=0,b=0,c=0,d=0;
  if (count==2 && !strcmp(tokens[0],"mode")) {
    const char *names[]={"start","ramp","sequence","manual","speed","deadband","saturation","gain","nonlinear","step","continued"};
    for (unsigned i=0;i<11;++i) if(!strcmp(tokens[1],names[i])) {candidate.mode=Mode(i);ok=true;}
  } else if (count==2 && !strcmp(tokens[0],"wheel") && strlen(tokens[1])==1) {
    candidate.wheel=tokens[1][0]; ok=true;
  } else if (count==2 && !strcmp(tokens[0],"format")) {
    if (!strcmp(tokens[1],"csv") || !strcmp(tokens[1],"plot") || !strcmp(tokens[1],"text")) {
      outputMode=!strcmp(tokens[1],"csv")?'c':!strcmp(tokens[1],"plot")?'p':'t';
      if(outputMode=='c') header(); else push(outputMode=='p'?"# FORMAT plot: Lrpm Rrpm\n":"# FORMAT text: short monitor, not analysis data\n");
      return;
    }
  } else if (count==2 && number(tokens[1],a)) {
    if (!strcmp(tokens[0],"sign")) {candidate.direction=a;ok=true;}
    else if (!strcmp(tokens[0],"pwm")) {candidate.pwm=a;ok=true;}
    else if (!strcmp(tokens[0],"limit")) {candidate.cap=a;ok=true;}
    else if (!strcmp(tokens[0],"repeats")) {candidate.repeats=a;ok=true;}
    else if (!strcmp(tokens[0],"interval") && a>=50 && a<=5000) {outputIntervalMs=uint32_t(a);push("# output interval updated\n");return;}
  } else if(count==4 && !strcmp(tokens[0],"range") && number(tokens[1],a)&&number(tokens[2],b)&&number(tokens[3],c)) {
    candidate.low=a;candidate.high=b;candidate.increment=c;ok=true;
  } else if(count==5 && number(tokens[1],a)&&number(tokens[2],b)&&number(tokens[3],c)&&number(tokens[4],d)) {
    if (a>=0&&b>=0&&c>=0&&d>=0 && !strcmp(tokens[0],"times")) {
      candidate.rest=a;candidate.settle=b;candidate.measure=c;candidate.cool=d;ok=true;
    } else if (a>=0&&b>=0&&c>=0&&d>=0 && !strcmp(tokens[0],"step")) {
      candidate.u0=a;candidate.u1=b;candidate.initial=c;candidate.finalHold=d;ok=true;
    }
  }
  Settings previous=experiment.cfg; experiment.cfg=candidate;
  if (!ok || !experiment.valid()) {experiment.cfg=previous;push("# rejected: invalid command/configuration; see README bounds\n");}
  else {reason="CONFIGURED";status();}
}

char commandBuffer[128]; unsigned commandLength=0; bool overflow=false;
void readCommands() {
  for(unsigned i=0;i<32 && Serial.available();++i) {
    char c=char(Serial.read());
    // '!' is an immediate abort, no newline required.
    if(c=='!') {halt("IMMEDIATE_ABORT",false);commandLength=0;overflow=false;continue;}
    if(c=='\r'||c=='\n') {
      if(overflow) push("# input too long; discarded\n");
      else if(commandLength) {commandBuffer[commandLength]=0;command(commandBuffer);}
      commandLength=0;overflow=false;
    } else if(commandLength<sizeof(commandBuffer)-1 && !overflow) commandBuffer[commandLength++]=c;
    else overflow=true;
  }
}

bool rawButton=HIGH, stableButton=HIGH, buttonArmed=false;
uint32_t buttonAt=0;
void button(uint32_t now) {
  bool level=digitalRead(Config::BUTTON_PIN);
  if(level!=rawButton) {rawButton=level;buttonAt=now;}
  if(now-buttonAt>=35) {
    if(rawButton==HIGH) buttonArmed=true;
    if(rawButton!=stableButton) {
      stableButton=rawButton;
      if(stableButton==LOW&&buttonArmed) {
        buttonArmed=false;
        if(experiment.active) halt("BUTTON",false); else startTrial();
      }
    }
  }
}
void setup() {
  Serial.begin(Config::SERIAL_BAUD);
  pinMode(Config::BUTTON_PIN,INPUT_PULLUP);
  busReady=bus.begin();
  bootAt=bootAttemptAt=millis();
  if(!busReady) {bootPending=false;reason="BUS_INIT_FAILED";}
  else reason="BOOT_WAIT";
  lastSample=lastZero=millis();
  push("# MotorLab v3 | 115200 | text monitor | send help\n");
  push("# BOOT checking robot; motors remain OFF\n");
}
void bootHandshake(uint32_t now) {
  if(now-bootAt<Config::BOOT_SETTLE_MS || now-bootAttemptAt<Config::BOOT_RETRY_MS) return;
  bootAttemptAt=now;
  // Unlike teacher's delay(50), allow the button/serial/output loop to keep running.
  bool stopped=bus.motors(0,0);
  uint32_t l,r;
  bool read=stopped && bus.encoders(l,r);
  if(stopped && read) {
    bool named=!Config::TEAM_NAME[0] || bus.team(Config::TEAM_NAME);
    if(named) {
      teamReady=Config::TEAM_NAME[0]!=0;
      bootPending=false;fault=false;reason="READY";
      appliedL=appliedR=0;lastSample=lastZero=millis();
      status();return;
    }
  }
  if(millis()-bootAt>=Config::BOOT_TIMEOUT_MS) {
    bootPending=false;halt("BOOT_HANDSHAKE",true);status();
  }
}
void loop() {
  uint32_t now=millis();
  if(experiment.active && now-lastSample>100) halt("SAMPLE_STALE",true);
  readCommands(); button(millis()); flushOutput();
  if(bootPending) {bootHandshake(millis());return;}
  now=millis();
  if(now-lastSample<Config::SAMPLE_MS) return;
  lastSample=now;
  bool wasActive=experiment.active;
  experiment.tick(now);
  if(experiment.restFault) {halt("REST_OR_BATCH_TIMEOUT",true);experiment.restFault=false;}
  int left=experiment.demand.left*Config::LEFT_MOTOR_SIGN;
  int right=experiment.demand.right*Config::RIGHT_MOTOR_SIGN;
  if(fault) left=right=0;
  if(left!=appliedL || right!=appliedR || (!experiment.active && now-lastZero>=100)) {
    if(!busReady || !bus.motors(left,right)) halt("MOTOR_WRITE",true);
    else {appliedL=left;appliedR=right;appliedAt=millis();if(!left&&!right)lastZero=appliedAt;}
  }
  uint32_t l=previousL,r=previousR;
  bool valid=busReady&&bus.encoders(l,r);
  uint32_t captured=millis(), dt=haveSample?captured-previousTime:0;
  if(!valid) {halt("ENCODER_READ",true);leftSpeed.reset();rightSpeed.reset();haveSample=false;rpmL=rpmR=0;}
  else {
    experiment.observe(haveSample&&(l!=previousL||r!=previousR),captured);
    rpmL=leftSpeed.update(l,captured,Config::COUNTS_PER_REV,Config::LEFT_ENCODER_SIGN);
    rpmR=rightSpeed.update(r,captured,Config::COUNTS_PER_REV,Config::RIGHT_ENCODER_SIGN);
    previousL=l;previousR=r;previousTime=captured;haveSample=true;
  }
  if(valid && !fault && (wasActive||experiment.active)) reportPhase();
  if(wasActive && !experiment.active && !fault) {
    char note[100];snprintf(note,sizeof note,"# DONE run=%lu motors=OFF\n",(unsigned long)experiment.batch);push(note);
  }
  if(!valid || fault) return;
  bool shouldLog=false;
  if(outputMode=='c') shouldLog=wasActive||experiment.active;
  else if(outputMode=='p') shouldLog=(wasActive||experiment.active) && captured-lastPlot>=outputIntervalMs;
  else shouldLog=experiment.active && captured-lastIdleLog>=outputIntervalMs && experiment.phase!=Phase::Rest && experiment.phase!=Phase::Cool;
  if(shouldLog) {
    lastPlot=lastIdleLog=captured;++sampleNumber;
    char row[ROW_SIZE];
    if(outputMode=='p') snprintf(row,sizeof row,"Lrpm:%.2f\tRrpm:%.2f\n",double(rpmL),double(rpmR));
    else if(outputMode=='t') snprintf(row,sizeof row,"%s | %s | L PWM=%d RPM=%.1f | R PWM=%d RPM=%.1f\n",
      modeName(),stageName(),appliedL,double(rpmL),appliedR,double(rpmR));
    else snprintf(row,sizeof row,"%lu,%lu,%s,%lu,%lu,%d,%d,%lld,%lld,%.2f,%.2f,%lu\n",
      (unsigned long)experiment.batch,(unsigned long)experiment.trial,stageName(),
      (unsigned long)captured,(unsigned long)dt,appliedL,appliedR,
      (long long)signedBits(l),(long long)signedBits(r),double(rpmL),double(rpmR),(unsigned long)appliedAt);
    if(!push(row) && experiment.active) halt("LOG_OVERFLOW",true);
  }
}
