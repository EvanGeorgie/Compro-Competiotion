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
  unsigned int timeouts;   // debug counter: read timeouts
  unsigned int noReads;    // debug counter: "no target" (255) readings
};

struct Motor {
  byte  in1, in2, en;
  float trim;
};

const int NUM_SENSORS = 3;
const int LEFT_S  = 0;
const int RIGHT_S = 1;
const int FRONT_S = 2;
Sensor sensors[NUM_SENSORS];

const int ir = 5;

Motor leftMotor  = {6, 7, 10, 1.00};
Motor rightMotor = {8, 9, 11, 1.15};

const bool DEBUG         = false;  
const bool TEST_GYRO     = false;  
const bool TEST_STRAIGHT = false;   
const bool TEST_TURN     = false;   

// ---- Gyro ----
const float GYRO_SIGN        = 1.0;   
const float GYRO_SCALE       = 65.5; 
const int   GYRO_CAL_SAMPLES = 500;  

const float HDG_KP       = 2.0;     // pwm per degree of heading error
const float HDG_KI       = 0.02;
const float HDG_KD       = 0.15;    // pwm per deg/s (damping)
const float HDG_I_MAX    = 300;     // integral clamp
const float HDG_MAX_CORR = 30;      // max left/right pwm difference

// Optional: gently aim away from the right wall using the ToF. 0 = OFF (ToF only solves the maze).
const float WALL_TRIM_GAIN    = 0.0;   // degrees of aim change per mm of error (try 0.1)
const float WALL_TRIM_MAX_DEG = 5.0;

// ---- Gyro turns (exact 90 / 180) ----
const float TURN_KP        = 2.5;   // pwm per degree of remaining error
const float TURN_KD        = 0.15;  // pwm per deg/s (slows the turn as it spins up)
const int   TURN_PEAK_PWM  = 100;   // top speed during the turn
const float TURN_TOL_DEG   = 2.0;   // stop when within this many degrees
const int   TURN_SETTLE_MS = 120;   // wait after braking, then re-check the angle
const int   TURN_ATTEMPTS  = 3;     // corrections if it stops outside the tolerance
const int   TURN_TIMEOUT_MS = 2000; // give up on one attempt after this long
const bool  TURN_AROUND_LEFT = true; // direction of the 180 at a dead end

// ---- Walls / sensing ----
const int GAP_MM         = 90;      // right reading at/above this = real opening
const int LEFT_GAP_MM    = 90;      // left reading at/above this = left is open
const int TRIGGER_MM     = 40;      // front distance that triggers a turn
const int NO_READING     = 255;
const int SIDE_TARGET_MM = 45;      // only used by the optional wall trim

// ---- Sensor dropout protection ----
const int SENSOR_TIMEOUT_MS = 80;   // max time one read may block
const int FRONT_LOST_MM     = 70;   // front "no reading" right after a value <= this = wall too close

// ---- Speed ----
int baseSpeed = 75;
const int MIN_PWM = 65;

// ---- Stops / turns ----
const int BRAKE_MS       = 60;
const int BACKUP_MS      = 0;       // reverse before a normal front-wall turn
const int SETTLE_MS      = 70;      // pause after turn so sensors give fresh data

// ---- Right-opening turn ----
const int APPROACH_MS    = 120;     // drive forward before turning at an opening
const int CLEAR_MS       = 250;     // drive forward after turning to clear corner
const byte GAP_CONFIRM   = 2;       // loops with a real opening before turning

// ---- Hit recovery (back up + turn to reset position) ----
const int  HIT_MM          = 20;    // front distance at/below this = "hit the wall"
const int  RECOVER_BACK_MS = 150;   // how long to reverse after a hit (try 100-250)

// =====================================================================

const byte MPU_ADDR = 0x68;
float gyroBias = 0;                 // deg/s, measured at startup
float yaw = 0;                      // deg, integrated heading (+ = left)
float gyroRate = 0;                 // deg/s (+ = turning left)
unsigned long lastGyroUs = 0;
float targetYaw = 0;                // heading the robot should hold
float hdgIntegral = 0;

int state = 0;
byte noRightCount = 0;
int lastFront = NO_READING;         // last valid front reading

void  sensorSetup(Sensor *s, byte xshut, byte addr, float offset, float alpha);
void  sensorBegin(Sensor *s);
void  recoverSensor(Sensor *s);
int   readSensor(Sensor *s);
int   readFresh(Sensor *s);
int   readFront();
bool  gyroInit();
bool  readGyroZ(float *dps);
void  calibrateGyro();
void  updateGyro();
float headingCorrection(float aim);
void  turnToHeading(float target);
void  turnBy(float deltaDeg);
void  chooseTurn(int dR, int dL);
void  motorRun(Motor *m, bool fwd, int pwm);
void  motorStop(Motor *m);
void  motorBrake(Motor *m);
void  forward(int LS, int RS);
void  stopMotors();
void  brakeMotors();
void  afterTurn();
void  waitMs(unsigned long ms);
bool  driveHeading(unsigned long ms);
void  backUp(unsigned long ms);
void  recoverFromHit();

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);             // VL6180X and MPU6050 both support 400 kHz
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

  // Gyro: must be found, then calibrated while the robot sits still
  if (!gyroInit()) {
    Serial.println("MPU6050 not found (check wiring / address 0x68)");
    while (true) { stopMotors(); delay(1000); }
  }
  Serial.println("Calibrating gyro - keep the robot still...");
  delay(500);
  calibrateGyro();
  Serial.print("Gyro bias (deg/s): "); Serial.println(gyroBias, 3);
}

void loop() {
  if (TEST_GYRO) {
    updateGyro();
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 100) {
      lastPrint = millis();
      Serial.print("yaw="); Serial.print(yaw, 1);
      Serial.print("  rate="); Serial.println(gyroRate, 1);
    }
    return;
  }

  if (TEST_STRAIGHT) {
    updateGyro();
    float c = headingCorrection(targetYaw);
    forward((int)(baseSpeed - c), (int)(baseSpeed + c));
    return;
  }

  if (TEST_TURN) {
    waitMs(2000);  turnBy( 90);
    waitMs(2000);  turnBy(-90);
    waitMs(2000);  turnBy( 180);
    waitMs(2000);  turnBy(-180);
    return;
  }

  if (digitalRead(ir) == 1) state = 1;
  if (state == 1) { stopMotors(); return; }

  // Straight driving: right + front ToF only (left is NOT read here)
  updateGyro();
  int dR = readSensor(&sensors[RIGHT_S]);
  int dF = readFront();
  updateGyro();

  if (dR < 0 || dF < 0) {
    stopMotors();
    return;
  }

  // Real opening on the right (far away / no reading), not just drifting off the wall
  bool rightOpen = (dR >= GAP_MM);

  if (DEBUG) {
    // once a second: how often each sensor timed out / returned "no target"
    static unsigned long lastDrop = 0;
    if (millis() - lastDrop >= 1000) {
      lastDrop = millis();
      Serial.print("DROPS timeout/noread");
      for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print(i == LEFT_S ? "  L:" : (i == RIGHT_S ? "  R:" : "  F:"));
        Serial.print(sensors[i].timeouts); Serial.print('/'); Serial.print(sensors[i].noReads);
      }
      Serial.println();
    }
    Serial.print("R="); Serial.print(dR);
    Serial.print(" F="); Serial.print(dF);
    Serial.print(" yaw="); Serial.print(yaw, 1);
    Serial.print(" tgt="); Serial.print(targetYaw, 0);
  }

  // ---- Hit: nose is against the wall -> back up, turn, resume ----
  if (dF <= HIT_MM) {
    if (DEBUG) Serial.println(" BRANCH=HIT_FRONT");
    recoverFromHit();
    return;
  }

  // ---- Front wall: stop, read right AND left, then turn 90 or 180 ----
  if (dF <= TRIGGER_MM) {
    brakeMotors();
    if (state) return;
    backUp(BACKUP_MS);
    if (state) return;

    // Robot is stopped: fresh readings, left sensor is used only here
    int fR = readFresh(&sensors[RIGHT_S]);
    int fL = readFresh(&sensors[LEFT_S]);
    updateGyro();
    if (fR < 0 || fL < 0) { stopMotors(); return; }

    if (DEBUG) {
      Serial.print(" BRANCH=FRONT fR="); Serial.print(fR);
      Serial.print(" fL="); Serial.println(fL);
    }

    chooseTurn(fR, fL);
    if (state) return;
    afterTurn();
    return;
  }

  // ---- Right side opened up (real gap): debounce, drive to opening, turn right ----
  noRightCount = rightOpen ? noRightCount + 1 : 0;
  if (noRightCount >= GAP_CONFIRM) {
    if (DEBUG) Serial.println(" BRANCH=GAP");
    noRightCount = 0;
    // If a front wall or sensor error shows up during the approach,
    // hand control back to loop() so the front branch brakes properly.
    if (!driveHeading(APPROACH_MS)) return;
    turnBy(-90);                    if (state) return;
    afterTurn();
    driveHeading(CLEAR_MS);
    return;
  }

  // ---- Straight: hold the target heading with the gyro ----
  float aim = targetYaw;
  if (WALL_TRIM_GAIN > 0 && dR < GAP_MM) {
    // too close to the right wall -> aim slightly left (+), too far -> slightly right (-)
    aim += constrain(WALL_TRIM_GAIN * (SIDE_TARGET_MM - dR), -WALL_TRIM_MAX_DEG, WALL_TRIM_MAX_DEG);
  }
  float corr = headingCorrection(aim);

  int LS = (int)(baseSpeed - corr);
  int RS = (int)(baseSpeed + corr);

  if (DEBUG) {
    Serial.print(" BRANCH=HOLD");
    Serial.print(" corr="); Serial.print(corr);
    Serial.print(" LS="); Serial.print(LS);
    Serial.print(" RS="); Serial.println(RS);
  }

  forward(LS, RS);
}

// ================= Maze decision =================

// Right open -> right 90.  Else left open -> left 90.  Else dead end -> 180.
void chooseTurn(int dR, int dL) {
  if (dR >= GAP_MM)           turnBy(-90);
  else if (dL >= LEFT_GAP_MM) turnBy( 90);
  else                        turnBy(TURN_AROUND_LEFT ? 180 : -180);
}

// Nose is against the wall: reverse, look right and left, turn.
void recoverFromHit() {
  brakeMotors();
  if (state) return;
  backUp(RECOVER_BACK_MS);
  if (state) return;

  waitMs(SETTLE_MS);
  int dR = readFresh(&sensors[RIGHT_S]);
  int dL = readFresh(&sensors[LEFT_S]);
  updateGyro();
  if (dR < 0 || dL < 0) { stopMotors(); return; }

  chooseTurn(dR, dL);
  if (state) return;
  afterTurn();
}

// ================= Gyro =================

bool gyroInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);                 // PWR_MGMT_1: wake up
  Wire.write(0x00);
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);                 // CONFIG: low-pass filter ~44 Hz
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);                 // GYRO_CONFIG: +-500 deg/s
  Wire.write(0x08);
  Wire.endTransmission();

  delay(100);
  return true;
}

bool readGyroZ(float *dps) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x47);                 // GYRO_ZOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)MPU_ADDR, 2) != 2) return false;
  uint8_t hi = Wire.read();
  uint8_t lo = Wire.read();
  *dps = (int16_t)((hi << 8) | lo) / GYRO_SCALE;
  return true;
}

// Average the gyro while the robot is still to find its zero-rate offset
void calibrateGyro() {
  float sum = 0;
  int n = 0;
  while (n < GYRO_CAL_SAMPLES) {
    float g;
    if (readGyroZ(&g)) { sum += g; n++; }
    delay(2);
  }
  gyroBias = sum / n;
  yaw = 0;
  targetYaw = 0;
  gyroRate = 0;
  lastGyroUs = micros();
}

// Call this often: integrates the turn rate into yaw
void updateGyro() {
  float g;
  unsigned long now = micros();
  if (!readGyroZ(&g)) { lastGyroUs = now; return; }
  float dt = (now - lastGyroUs) * 1.0e-6f;
  lastGyroUs = now;
  if (dt > 0.2f) dt = 0.2f;
  gyroRate = GYRO_SIGN * (g - gyroBias);
  yaw += gyroRate * dt;
}

// PD(+I) on heading. Returns a left/right pwm difference:
// positive = steer left (left wheel slower, right wheel faster).
float headingCorrection(float aim) {
  float err = aim - yaw;
  hdgIntegral = constrain(hdgIntegral + err, -HDG_I_MAX, HDG_I_MAX);
  float out = HDG_KP * err + HDG_KI * hdgIntegral - HDG_KD * gyroRate;
  return constrain(out, -HDG_MAX_CORR, HDG_MAX_CORR);
}

// Turn on the spot until yaw reaches target (within TURN_TOL_DEG),
// then brake, wait, and correct again if it stopped short or overshot.
void turnToHeading(float target) {
  for (int attempt = 0; attempt < TURN_ATTEMPTS; attempt++) {
    unsigned long start = millis();

    while (millis() - start < (unsigned long)TURN_TIMEOUT_MS) {
      if (digitalRead(ir) == 1) { state = 1; stopMotors(); return; }
      updateGyro();

      float err = target - yaw;                  // + = need to turn left
      if (fabs(err) <= TURN_TOL_DEG) break;

      float dir = (err > 0) ? 1.0 : -1.0;
      // proportional to remaining angle, reduced while spinning fast toward the target
      float m = TURN_KP * fabs(err) - TURN_KD * gyroRate * dir;
      m = constrain(m, (float)MIN_PWM, (float)TURN_PEAK_PWM);

      bool left = (dir > 0);
      motorRun(&leftMotor,  !left, (int)(m * leftMotor.trim));   // left turn: left wheel back
      motorRun(&rightMotor,  left, (int)(m * rightMotor.trim));  //            right wheel forward
    }

    brakeMotors();                               // active brake (yaw keeps integrating)
    if (state) return;
    waitMs(TURN_SETTLE_MS);                      // let it stop rocking
    if (state) return;
    if (fabs(target - yaw) <= TURN_TOL_DEG) return;
  }
}

// Turn by deltaDeg relative to the held heading (+ = left, - = right).
// The target is absolute, so any error left over from the last turn is corrected too.
void turnBy(float deltaDeg) {
  targetYaw += deltaDeg;
  turnToHeading(targetYaw);
}

// ================= Sensors =================

void sensorSetup(Sensor *s, byte xshut, byte addr, float offset, float alpha) {
  s->xshut     = xshut;
  s->addr      = addr;
  s->offset    = offset;
  s->failCount = 0;
  s->alpha     = alpha;
  s->filt      = -1;
  s->timeouts  = 0;
  s->noReads   = 0;
}

void sensorBegin(Sensor *s) {
  s->dev.init();
  s->dev.configureDefault();
  s->dev.setTimeout(SENSOR_TIMEOUT_MS);
  s->dev.setAddress(s->addr);
  s->dev.startRangeContinuous(20);   // continuous mode: new reading every 20 ms
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
    s->timeouts++;
    if (++s->failCount >= 3) {
      recoverSensor(s);
      s->failCount = 0;
    }
    return -1;
  }
  s->failCount = 0;
  if (raw >= 255) {
    s->noReads++;
    s->filt = -1;            // don't blend "no reading" into the filter
    return NO_READING;
  }

  float v = raw - s->offset;
  if (v < 0) v = 0;

  if (s->filt < 0) s->filt = v;
  else             s->filt = s->alpha * v + (1.0 - s->alpha) * s->filt;

  return (int)round(s->filt);
}

// Fresh, unfiltered reading (robot should be stopped). The first read may be an
// old sample (the sensor keeps measuring in the background), so it is thrown
// away, and the filter is cleared so the result is not blended with old data.
int readFresh(Sensor *s) {
  readSensor(s);
  s->filt = -1;
  return readSensor(s);
}

// Front reading with dropout protection. A wall in front cannot vanish while
// driving straight, so "no reading" right after a close value is treated as
// "wall too close" (sensor lost it) instead of "path is clear".
int readFront() {
  int d = readSensor(&sensors[FRONT_S]);
  if (d < 0) return d;
  if (d >= NO_READING) {
    if (lastFront <= FRONT_LOST_MM) return TRIGGER_MM;
    return d;
  }
  lastFront = d;
  return d;
}

// ================= State after a turn =================

// Clear stale state after a turn (distances change a lot)
void afterTurn() {
  hdgIntegral = 0;
  noRightCount = 0;
  lastFront = NO_READING;          // new heading: forget the old front wall
  waitMs(SETTLE_MS);
  readSensor(&sensors[RIGHT_S]);   // throw away the stale reading
  sensors[RIGHT_S].filt = -1;
  readSensor(&sensors[FRONT_S]);
  sensors[FRONT_S].filt = -1;
  updateGyro();
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

// Drive straight for ms, holding the target heading. Returns true if the full
// time was driven; false if stopped early (front wall, sensor error, or IR stop).
bool driveHeading(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (digitalRead(ir) == 1) { state = 1; stopMotors(); return false; }

    updateGyro();
    int dF = readFront();
    updateGyro();
    if (dF < 0)           { stopMotors(); return false; }
    if (dF <= TRIGGER_MM) { stopMotors(); return false; }

    float c = headingCorrection(targetYaw);
    forward((int)(baseSpeed - c), (int)(baseSpeed + c));
  }
  return true;
}

void backUp(unsigned long ms) {
  if (ms == 0) return;
  motorRun(&leftMotor,  false, MIN_PWM + 10);
  motorRun(&rightMotor, false, MIN_PWM + 10);
  waitMs(ms);
  brakeMotors();
}

void stopMotors() {
  motorStop(&leftMotor);
  motorStop(&rightMotor);
}

void brakeMotors() {
  motorBrake(&leftMotor);
  motorBrake(&rightMotor);
  waitMs(BRAKE_MS);                // keeps integrating yaw while braking
  stopMotors();
}

// Wait without losing the gyro: keeps updating yaw, stops early on the IR stop.
void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (digitalRead(ir) == 1) { state = 1; return; }
    updateGyro();
  }
}
