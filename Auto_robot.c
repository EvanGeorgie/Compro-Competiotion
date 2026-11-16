#include <Wire.h>
#include <VL6180X.h>

struct Sensor {
  VL6180X dev;
  byte    xshut;
  byte    addr;
  float   offset;
  byte    failCount;
  float   alpha;      // low-pass filter: 1.0 = no filtering, lower = smoother
  float   filt;       // filtered value (-1 = not initialised)
};

struct Motor {
  byte  in1, in2, en;
  float trim;
};

struct PID {
  float kp, ki, kd;
  float integral, lastError;
  float integralMax, maxCorr;
};

const int NUM_SENSORS = 3;
const int LEFT_S  = 0;
const int RIGHT_S = 1;
const int FRONT_S = 2;
Sensor sensors[NUM_SENSORS];

const int ir = 5;

Motor leftMotor  = {6, 7, 10, 1.00};
Motor rightMotor = {8, 9, 11, 1.00};

const bool DEBUG         = false;
const bool TEST_STRAIGHT = false;
const int SIDE_WALL_MM   = 100;
const int TRIGGER_MM     = 80;
const int NO_READING     = 255;

// ---- Turn tuning (retune these on the robot) ----
const int TURN_90_MS     = 377;   // was 377; ramped profile needs a bit longer
const int TURN_180_MS    = 650;
const int BRAKE_MS       = 60;
const int TURN_PEAK_PWM  = 100;   // top speed during the turn
const float RAMP_UP      = 0.25;  // fraction of turn spent accelerating
const float RAMP_DOWN    = 0.35;  // fraction of turn spent decelerating
const int APPROACH_MS    = 120;   // drive forward before turning at an opening
const int CLEAR_MS       = 250;   // drive forward after turning to clear corner
const byte GAP_CONFIRM   = 3;     // consecutive loops with no right wall before turning

int baseSpeed = 90;
const int MIN_PWM  = 65;

const float CENTER_BIAS    = 0;
const int   SIDE_TARGET_MM = 45;

// kd lowered from 0.6 to reduce twitching from noisy distance data
PID pid = {1.0, 0.05, 0.7, 0, 0, 100, 20};
int lastMode = 0;
int state = 0;
byte noRightCount = 0;

void  sensorSetup(Sensor *s, byte xshut, byte addr, float offset, float alpha);
void  sensorBegin(Sensor *s);
void  recoverSensor(Sensor *s);
int   readSensor(Sensor *s);
void  motorRun(Motor *m, bool fwd, int pwm);
void  motorStop(Motor *m);
void  motorBrake(Motor *m);
void  forward(int LS, int RS);
void  smoothPivot(bool turnRight, unsigned long totalMs);
void  stopMotors();
void  brakeMotors();
void  pidReset(PID *p);
float pidUpdate(PID *p, float error);
void  resetPID();
void  afterTurn();
void  waitMs(unsigned long ms);

void setup() {
  Serial.begin(115200);
  Wire.begin();
  #if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);
  #endif

  pinMode(ir, INPUT);
  Motor *motors[2] = {&leftMotor, &rightMotor};
  for (int i = 0; i < 2; i++) {
    pinMode(motors[i]->in1, OUTPUT);
    pinMode(motors[i]->in2, OUTPUT);
    pinMode(motors[i]->en,  OUTPUT);
  }
  stopMotors();

  // Side sensors filtered (alpha 0.4); front unfiltered for fast wall detection
  sensorSetup(&sensors[LEFT_S],  12, 0x30, -34.40, 0.4);
  sensorSetup(&sensors[RIGHT_S],  3, 0x31,  -1.20, 0.4);
  sensorSetup(&sensors[FRONT_S],  4, 0x32,   0.00, 1.0);

  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(sensors[i].xshut, OUTPUT);
    digitalWrite(sensors[i].xshut, LOW);
  }
  delay(50);

  for (int i = 0; i < NUM_SENSORS; i++) {
    digitalWrite(sensors[i].xshut, HIGH);
    delay(50);
    sensorBegin(&sensors[i]);
  }
}

void loop() {
  if (TEST_STRAIGHT) {
    forward(baseSpeed, baseSpeed);
    return;
  }

  if (digitalRead(ir) == 1) state = 1;
  if (state == 1) { stopMotors(); return; }

  int dL = readSensor(&sensors[LEFT_S]);
  int dR = readSensor(&sensors[RIGHT_S]);
  int dF = readSensor(&sensors[FRONT_S]);

  if (dL < 0 || dR < 0 || dF < 0) {
    stopMotors();
    return;
  }

  bool leftWall  = (dL <= SIDE_WALL_MM);
  bool rightWall = (dR <= SIDE_WALL_MM);

  if (DEBUG) {
    Serial.print("L="); Serial.print(dL);
    Serial.print(" R="); Serial.print(dR);
    Serial.print(" F="); Serial.print(dF);
  }

  // ---- Front wall: brake, then smooth turn ----
  if (dF <= TRIGGER_MM) {
    brakeMotors();
    if (state) return;

    if (dR > SIDE_WALL_MM)      smoothPivot(true,  TURN_90_MS);
    else if (dL > SIDE_WALL_MM) smoothPivot(false, TURN_90_MS);
    else                        smoothPivot(true,  TURN_180_MS);

    afterTurn();
    return;
  }

  // ---- Right wall opened up: debounce, drive to opening, turn without stopping ----
  noRightCount = rightWall ? 0 : noRightCount + 1;
  if (noRightCount >= GAP_CONFIRM) {
    noRightCount = 0;
    forward(baseSpeed, baseSpeed);  waitMs(APPROACH_MS);  if (state) return;
    smoothPivot(true, TURN_90_MS);                        if (state) return;
    afterTurn();
    forward(baseSpeed, baseSpeed);  waitMs(CLEAR_MS);
    return;
  }
  if (!rightWall) {
    // Gap not confirmed yet: keep driving straight
    forward(baseSpeed, baseSpeed);
    return;
  }

  // ---- Wall following with PID ----
  int mode = leftWall ? 1 : 2;
  if (mode != lastMode) {
    resetPID();
    lastMode = mode;
  }

  float error;
  if (mode == 1) error = (dL - dR) - CENTER_BIAS;
  else           error = SIDE_TARGET_MM - dR;

  float correction = pidUpdate(&pid, error);

  int LS = (int)(baseSpeed - correction);
  int RS = (int)(baseSpeed + correction);

  if (DEBUG) {
    Serial.print(" err="); Serial.print(error);
    Serial.print(" corr="); Serial.print(correction);
    Serial.print(" LS="); Serial.print(LS);
    Serial.print(" RS="); Serial.println(RS);
  }

  forward(LS, RS);
}

// ================= Sensors =================

void sensorSetup(Sensor *s, byte xshut, byte addr, float offset, float alpha) {
  s->xshut     = xshut;
  s->addr      = addr;
  s->offset    = offset;
  s->failCount = 0;
  s->alpha     = alpha;
  s->filt      = -1;
}

void sensorBegin(Sensor *s) {
  s->dev.init();
  s->dev.configureDefault();
  s->dev.setTimeout(500);
  s->dev.setAddress(s->addr);
  s->dev.startRangeContinuous(50);   // continuous mode: new reading every 50 ms
  s->filt = -1;
}

void recoverSensor(Sensor *s) {
  stopMotors();
  digitalWrite(s->xshut, LOW);   delay(20);
  digitalWrite(s->xshut, HIGH);  delay(50);
  s->dev.setAddress(0x29);
  sensorBegin(s);
}

int readSensor(Sensor *s) {
  int raw = s->dev.readRangeContinuousMillimeters();
  if (s->dev.timeoutOccurred()) {
    if (++s->failCount >= 3) {
      recoverSensor(s);
      s->failCount = 0;
    }
    return -1;
  }
  s->failCount = 0;
  if (raw >= 255) {
    s->filt = -1;            // don't blend "no reading" into the filter
    return NO_READING;
  }

  float v = raw - s->offset;
  if (v < 0) v = 0;

  if (s->filt < 0) s->filt = v;
  else             s->filt = s->alpha * v + (1.0 - s->alpha) * s->filt;

  return (int)round(s->filt);
}

// ================= PID =================

void pidReset(PID *p) {
  p->integral  = 0;
  p->lastError = 0;
}

float pidUpdate(PID *p, float error) {
  p->integral = constrain(p->integral + error, -p->integralMax, p->integralMax);
  float out = p->kp * error + p->ki * p->integral + p->kd * (error - p->lastError);
  out = constrain(out, -p->maxCorr, p->maxCorr);
  p->lastError = error;
  return out;
}

void resetPID() {
  pidReset(&pid);
  lastMode = 0;
}

// Clear stale state after a turn (distances change a lot)
void afterTurn() {
  resetPID();
  noRightCount = 0;
  for (int i = 0; i < NUM_SENSORS; i++) sensors[i].filt = -1;
}

// ================= Motors =================

void motorRun(Motor *m, bool fwd, int pwm) {
  digitalWrite(m->in1, fwd);
  digitalWrite(m->in2, !fwd);
  analogWrite(m->en, constrain(pwm, 0, 255));
}

void motorStop(Motor *m) {
  digitalWrite(m->in1, 0);
  digitalWrite(m->in2, 0);
  analogWrite(m->en, 0);
}

void motorBrake(Motor *m) {
  digitalWrite(m->in1, 1);
  digitalWrite(m->in2, 1);
  analogWrite(m->en, 255);
}

void forward(int LS, int RS) {
  int a = constrain((int)(LS * leftMotor.trim),  MIN_PWM, 255);
  int b = constrain((int)(RS * rightMotor.trim), MIN_PWM, 255);
  motorRun(&leftMotor,  true, a);
  motorRun(&rightMotor, true, b);
}

// Pivot with accelerate / cruise / decelerate profile, then active brake
void smoothPivot(bool turnRight, unsigned long totalMs) {
  unsigned long start = millis();

  while (true) {
    unsigned long t = millis() - start;
    if (t >= totalMs) break;
    if (digitalRead(ir) == 1) { state = 1; stopMotors(); return; }

    float phase = (float)t / totalMs;
    float scale = 1.0;
    if (phase < RAMP_UP)               scale = phase / RAMP_UP;
    else if (phase > 1.0 - RAMP_DOWN)  scale = (1.0 - phase) / RAMP_DOWN;

    int pwm = MIN_PWM + (int)((TURN_PEAK_PWM - MIN_PWM) * scale);
    motorRun(&leftMotor,  turnRight,  (int)(pwm * leftMotor.trim));
    motorRun(&rightMotor, !turnRight, (int)(pwm * rightMotor.trim));
  }
  brakeMotors();
}

void stopMotors() {
  motorStop(&leftMotor);
  motorStop(&rightMotor);
}

void brakeMotors() {
  motorBrake(&leftMotor);
  motorBrake(&rightMotor);
  delay(BRAKE_MS);
  stopMotors();
}

void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (digitalRead(ir) == 1) { state = 1; return; }
  }
}
