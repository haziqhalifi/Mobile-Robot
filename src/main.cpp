/*
  ================================================================
  KS0560 Mecanum Car - LINE FOLLOW TO INTERSECTION 3
  ================================================================
  LOGIC:
    - Button 1: Follow the straight line and stop at intersection 3.
    - Button 2: Follow the straight line, stop at intersection 3, then turn right 90 degrees.
    - Button 3: Follow the straight line, stop at intersection 3, turn right 90 degrees, then strafe left.
    - Button 4: Drive forward by time, turn right by time, then strafe left by time.
    - Button 5: Go straight 2 intersections, turn left to centre line, go straight 2 intersections, turn right to centre line, then go straight 2 intersections.
    - Button 6: Same as button 5, then turn 180 degrees.
    - Button 7: Button 6 route, then reverse back to the initial position without the last 180-degree turn.
    - Left arrow: Manual strafe left.
    - Button 0: Stop manual movement.
    - Button *: Emergency stop while moving.
  ================================================================
*/

#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.h>

// Hardware pins
#define RECV_PIN        A3
#define SENSOR_LEFT     A0
#define SENSOR_MID      A1
#define SENSOR_RIGHT    A2

// External library speed variables
extern uint8_t speed_Upper_L;
extern uint8_t speed_Lower_L;
extern uint8_t speed_Upper_R;
extern uint8_t speed_Lower_R;

// IR codes
#define BTN_1           0xFF6897
#define BTN_2           0xFF9867
#define BTN_3           0xFFB04F
#define BTN_4           0xFF30CF
#define BTN_5           0xFF18E7
#define BTN_6           0xFF7A85
#define BTN_7           0xFF10EF
#define BTN_LEFT        0xFF22DD
#define BTN_0           0xFF4AB5
#define BTN_STAR        0xFF42BD
#define BTN_REPEAT      0xFFFFFFFF

// Speeds
#define SPEED_FORWARD   60  // 60
#define SPEED_TURN      45
#define SPEED_ROTATE    60  //60
#define SPEED_STRAFE    80  //55
#define SPEED_MIN       35

// Line configuration
#define BLACK_LINE      HIGH
#define TARGET_BLOCKS   3
#define FORWARD_STOP_OFFSET_MS 120
#define BUTTON5_FORWARD_OFFSET_MS 120
#define AFTER_TURN_ALIGN_MS 500
#define STOP_SENSOR_RIGHT_TURN SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN SENSOR_LEFT
#define TURN_BLIND_MS   150
#define RIGHT_TURN_MIDDLE_BLIND_MS 600
#define BUTTON5_TURN_BLIND_MS 500
#define BUTTON5_LEFT_TURN_BLIND_MS 600
#define STRAFE_LEFT_MS  2000
#define TARGET_STRAFE_LINES 2
#define STRAFE_START_IGNORE_MS 300
#define STRAFE_LINE_MIN_GAP_MS 250
#define STRAFE_AFTER_LINE_MS 350
#define TIMED_FORWARD_MS 3000
#define TIMED_RIGHT_TURN_MS 650

mecanumCar car(3, 2);
IRrecv irrecv(RECV_PIN);
decode_results results;

// ================================================================
//  HELPERS
// ================================================================
void setMotorSpeed(uint8_t spd) {
  speed_Upper_L = spd;
  speed_Lower_L = spd;
  speed_Upper_R = spd;
  speed_Lower_R = spd;
}

void restoreIR() {
  car.Stop();
  irrecv.enableIRIn();
}

bool emergencyStopPressed() {
  if (!irrecv.decode(&results)) {
    return false;
  }

  bool stopped = results.value == BTN_STAR;
  irrecv.resume();

  if (stopped) {
    Serial.println(F("E-STOP PRESSED!"));
  }

  return stopped;
}

bool isIntersection(uint8_t leftSensor, uint8_t middleSensor, uint8_t rightSensor) {
  return leftSensor == BLACK_LINE &&
         middleSensor == BLACK_LINE &&
         rightSensor == BLACK_LINE;
}

void driveStraight(uint8_t spd) {
  setMotorSpeed(spd);
  car.Advance();
}

void correctLeft(uint8_t spd) {
  setMotorSpeed(spd);
  car.Turn_Left();
}

void correctRight(uint8_t spd) {
  setMotorSpeed(spd);
  car.Turn_Right();
}

void manualStrafeLeft() {
  setMotorSpeed(SPEED_STRAFE);
  car.L_Move();
  Serial.println(F("[COMMAND] Strafing Left."));
}

void manualStop() {
  car.Stop();
  Serial.println(F("[COMMAND] Halt."));
}

// ================================================================
//  FOLLOW LINE AND STOP AT INTERSECTION 3
// ================================================================
bool moveForwardBlocks(int targetBlocks, uint8_t startSpeed, unsigned long stopOffsetMs = 0) {
  int junctionCount = 0;
  bool onJunction = true;
  bool completed = false;

  Serial.print(F("\n--- Following Line to Intersection "));
  Serial.print(targetBlocks);
  Serial.print(F(" at Speed "));
  Serial.print(startSpeed);
  Serial.println(F(" ---"));

  while (junctionCount < targetBlocks) {
    if (emergencyStopPressed()) {
      break;
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    if (isIntersection(L, M, R)) {
      if (!onJunction) {
        junctionCount++;
        onJunction = true;

        Serial.print(F("Intersection: "));
        Serial.println(junctionCount);

        if (junctionCount >= targetBlocks) {
          completed = true;
          break;
        }
      }

      driveStraight(startSpeed);
    }
    else if (L == LOW && M == HIGH && R == LOW) {
      onJunction = false;
      driveStraight(startSpeed);
    }
    else if (L == HIGH && M == HIGH && R == LOW) {
      onJunction = false;
      correctLeft(SPEED_TURN);
    }
    else if (L == LOW && M == HIGH && R == HIGH) {
      onJunction = false;
      correctRight(SPEED_TURN);
    }
    else if (L == HIGH && M == LOW && R == LOW) {
      onJunction = false;
      correctLeft(SPEED_TURN);
    }
    else if (L == LOW && M == LOW && R == HIGH) {
      onJunction = false;
      correctRight(SPEED_TURN);
    }
    else {
      onJunction = false;
      driveStraight(SPEED_MIN);
    }
  }

  if (completed && stopOffsetMs > 0) {
    unsigned long startTime = millis();
    while (millis() - startTime < stopOffsetMs) {
      if (emergencyStopPressed()) {
        completed = false;
        break;
      }

      driveStraight(startSpeed);
    }
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- Movement Completed. ---"));
  restoreIR();
  return completed;
}

bool followLineFor(unsigned long durationMs, uint8_t startSpeed) {
  Serial.print(F("\n--- Aligning on Line for "));
  Serial.print(durationMs);
  Serial.println(F(" ms ---"));

  unsigned long startTime = millis();
  while (millis() - startTime < durationMs) {
    if (emergencyStopPressed()) {
      car.Stop();
      restoreIR();
      return false;
    }

    uint8_t L = digitalRead(SENSOR_LEFT);
    uint8_t M = digitalRead(SENSOR_MID);
    uint8_t R = digitalRead(SENSOR_RIGHT);

    if (L == LOW && M == HIGH && R == LOW) {
      driveStraight(startSpeed);
    }
    else if (L == HIGH && R == LOW) {
      correctLeft(SPEED_TURN);
    }
    else if (L == LOW && R == HIGH) {
      correctRight(SPEED_TURN);
    }
    else if (M == HIGH) {
      driveStraight(startSpeed);
    }
    else {
      driveStraight(SPEED_MIN);
    }
  }

  car.Stop();
  delay(150);
  Serial.println(F("--- Line Alignment Completed. ---"));
  restoreIR();
  return true;
}

// ================================================================
//  TURN 90 DEGREES RIGHT
// ================================================================
bool rotateRight90WithStopSensor(uint8_t stopSensor, const __FlashStringHelper *label, unsigned long blindMs) {
  Serial.print(F("\n--- Rotating 90 Degrees Right until "));
  Serial.print(label);
  Serial.println(F(" Sensor Finds Line ---"));

  unsigned long startTime = millis();
  while (millis() - startTime < blindMs) {
    if (emergencyStopPressed()) {
      restoreIR();
      return false;
    }

    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();
  }

  while (true) {
    if (emergencyStopPressed()) {
      restoreIR();
      return false;
    }

    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();

    if (digitalRead(stopSensor) == BLACK_LINE) {
      break;
    }
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- Right Turn Completed. ---"));
  restoreIR();
  return true;
}

bool rotateRight90() {
  return rotateRight90WithStopSensor(STOP_SENSOR_RIGHT_TURN, F("Right"), TURN_BLIND_MS);
}

bool rotateRight90Button5() {
  return rotateRight90WithStopSensor(STOP_SENSOR_RIGHT_TURN, F("Right"), BUTTON5_TURN_BLIND_MS);
}

bool rotateRight90Button5UntilMiddle() {
  return rotateRight90WithStopSensor(SENSOR_MID, F("Middle"), BUTTON5_TURN_BLIND_MS);
}

bool rotateRight90UntilMiddle() {
  return rotateRight90WithStopSensor(SENSOR_MID, F("Middle"), RIGHT_TURN_MIDDLE_BLIND_MS);
}

// ================================================================
//  TURN 90 DEGREES LEFT
// ================================================================
bool rotateLeft90WithStopSensor(uint8_t stopSensor, const __FlashStringHelper *label, unsigned long blindMs) {
  Serial.print(F("\n--- Rotating 90 Degrees Left until "));
  Serial.print(label);
  Serial.println(F(" Sensor Finds Line ---"));

  unsigned long startTime = millis();
  while (millis() - startTime < blindMs) {
    if (emergencyStopPressed()) {
      restoreIR();
      return false;
    }

    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();
  }

  while (true) {
    if (emergencyStopPressed()) {
      restoreIR();
      return false;
    }

    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Left();

    if (digitalRead(stopSensor) == BLACK_LINE) {
      break;
    }
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- Left Turn Completed. ---"));
  restoreIR();
  return true;
}

bool rotateLeft90() {
  return rotateLeft90WithStopSensor(STOP_SENSOR_LEFT_TURN, F("Left"), TURN_BLIND_MS);
}

bool rotateLeft90Button5() {
  return rotateLeft90WithStopSensor(SENSOR_MID, F("Middle"), BUTTON5_LEFT_TURN_BLIND_MS);
}

// ================================================================
//  TURN 180 DEGREES RIGHT
// ================================================================
bool rotateRight180() {
  Serial.println(F("\n--- Rotating 180 Degrees Right ---"));

  if (!rotateRight90()) {
    return false;
  }

  if (!rotateRight90()) {
    return false;
  }

  Serial.println(F("--- 180 Turn Completed. ---"));
  return true;
}

bool rotateRight180Button5() {
  Serial.println(F("\n--- Rotating 180 Degrees Right ---"));

  if (!rotateRight90Button5()) {
    return false;
  }

  if (!rotateRight90Button5()) {
    return false;
  }

  Serial.println(F("--- 180 Turn Completed. ---"));
  return true;
}

// ================================================================
//  STRAFE LEFT FOR A FIXED TIME
// ================================================================
bool strafeLeftFor(unsigned long durationMs) {
  Serial.print(F("\n--- Strafing Left for "));
  Serial.print(durationMs);
  Serial.println(F(" ms ---"));

  unsigned long startTime = millis();
  while (millis() - startTime < durationMs) {
    if (emergencyStopPressed()) {
      restoreIR();
      return false;
    }

    setMotorSpeed(SPEED_STRAFE);
    car.L_Move();
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- Left Strafe Completed. ---"));
  restoreIR();
  return true;
}

bool finishStrafeLeftFor(unsigned long durationMs) {
  unsigned long startTime = millis();
  while (millis() - startTime < durationMs) {
    if (emergencyStopPressed()) {
      restoreIR();
      return false;
    }

    setMotorSpeed(SPEED_STRAFE);
    car.L_Move();
  }

  return true;
}

// ================================================================
//  STRAFE LEFT AND COUNT LINES WITH MIDDLE SENSOR ONLY
// ================================================================
bool strafeLeftLines(int targetLines) {
  int lineCount = 0;
  bool onLine = true;
  bool completed = false;
  unsigned long lastLineTime = 0;

  Serial.print(F("\n--- Strafing Left to Cross "));
  Serial.print(targetLines);
  Serial.println(F(" Lines with Middle Sensor ---"));

  unsigned long ignoreStart = millis();
  while (millis() - ignoreStart < STRAFE_START_IGNORE_MS) {
    if (emergencyStopPressed()) {
      car.Stop();
      restoreIR();
      return false;
    }

    setMotorSpeed(SPEED_STRAFE);
    car.L_Move();
  }

  onLine = digitalRead(SENSOR_MID) == BLACK_LINE;
  lastLineTime = millis();

  while (lineCount < targetLines) {
    if (emergencyStopPressed()) {
      break;
    }

    uint8_t M = digitalRead(SENSOR_MID);

    setMotorSpeed(SPEED_STRAFE);
    car.L_Move();

    if (M == BLACK_LINE) {
      if (!onLine && millis() - lastLineTime >= STRAFE_LINE_MIN_GAP_MS) {
        lineCount++;
        onLine = true;
        lastLineTime = millis();

        Serial.print(F("Strafe line: "));
        Serial.println(lineCount);

        if (lineCount >= targetLines) {
          completed = finishStrafeLeftFor(STRAFE_AFTER_LINE_MS);
          break;
        }
      }
      else {
        onLine = true;
      }
    }
    else {
      onLine = false;
    }
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- Strafe Line Count Completed. ---"));
  restoreIR();
  return completed;
}

// ================================================================
//  TIMED MOVEMENT WITHOUT LINE TRACING
// ================================================================
bool driveStraightFor(unsigned long durationMs) {
  Serial.print(F("\n--- Driving Straight for "));
  Serial.print(durationMs);
  Serial.println(F(" ms ---"));

  unsigned long startTime = millis();
  while (millis() - startTime < durationMs) {
    if (emergencyStopPressed()) {
      restoreIR();
      return false;
    }

    driveStraight(SPEED_FORWARD);
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- Timed Forward Completed. ---"));
  restoreIR();
  return true;
}

bool rotateRightFor(unsigned long durationMs) {
  Serial.print(F("\n--- Timed Right Turn for "));
  Serial.print(durationMs);
  Serial.println(F(" ms ---"));

  unsigned long startTime = millis();
  while (millis() - startTime < durationMs) {
    if (emergencyStopPressed()) {
      restoreIR();
      return false;
    }

    setMotorSpeed(SPEED_ROTATE);
    car.Turn_Right();
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- Timed Right Turn Completed. ---"));
  restoreIR();
  return true;
}

// ================================================================
//  ROUTES
// ================================================================
void routeToIntersection3() {
  moveForwardBlocks(TARGET_BLOCKS, SPEED_FORWARD, FORWARD_STOP_OFFSET_MS);
}

bool routeToIntersection3AndTurnRight() {
  if (!moveForwardBlocks(TARGET_BLOCKS, SPEED_FORWARD, FORWARD_STOP_OFFSET_MS)) {
    return false;
  }

  return rotateRight90UntilMiddle();
}

void routeToIntersection3TurnRightAndStrafeLeft() {
  if (routeToIntersection3AndTurnRight() &&
      followLineFor(AFTER_TURN_ALIGN_MS, SPEED_FORWARD)) {
    strafeLeftFor(STRAFE_LEFT_MS);
  }
}

void routeTimedForwardTurnRightAndStrafeLeft() {
  if (driveStraightFor(TIMED_FORWARD_MS) && rotateRightFor(TIMED_RIGHT_TURN_MS)) {
    strafeLeftFor(STRAFE_LEFT_MS);
  }
}

bool routeButton5Path() {
  if (!moveForwardBlocks(2, SPEED_FORWARD, BUTTON5_FORWARD_OFFSET_MS)) {
    return false;
  }

  if (!rotateLeft90Button5()) {
    return false;
  }

  if (!moveForwardBlocks(2, SPEED_FORWARD, BUTTON5_FORWARD_OFFSET_MS)) {
    return false;
  }

  if (!rotateRight90Button5UntilMiddle()) {
    return false;
  }

  return moveForwardBlocks(2, SPEED_FORWARD, BUTTON5_FORWARD_OFFSET_MS);
}

void routeLeftAt2RightAt3() {
  routeButton5Path();
}

bool routeButton6Path() {
  if (routeButton5Path()) {
    return rotateRight180Button5();
  }

  return false;
}

void routeLeftAt2RightAt3ThenStrafeLeft() {
  routeButton6Path();
}

void routeButton6AndReturnToStart() {
  if (routeButton6Path()) {
    routeButton5Path();
  }
}

// ================================================================
//  SETUP & LOOP
// ================================================================
void setup() {
  Serial.begin(9600);

  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_MID, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  car.Init();
  irrecv.enableIRIn();

  Serial.println(F("Ready."));
  Serial.println(F("Press '1' to follow the line and stop at intersection 3."));
  Serial.println(F("Press '2' to stop at intersection 3, then turn right 90 degrees."));
  Serial.println(F("Press '3' to do button 2, then strafe left."));
  Serial.println(F("Press '4' to drive timed forward, timed right turn, then strafe left."));
  Serial.println(F("Press '5' to go 2 intersections, turn left, go 2, turn right, then go 2."));
  Serial.println(F("Press '6' to do button 5 path, then turn 180 degrees."));
  Serial.println(F("Press '7' to do button 6, then reverse back without the last 180 turn."));
  Serial.println(F("Press left arrow to manual strafe left."));
  Serial.println(F("Press '0' to stop manual movement."));
  Serial.println(F("Press '*' for emergency stop while moving."));
}

void loop() {
  if (!irrecv.decode(&results)) {
    return;
  }

  unsigned long code = results.value;

  if (code == BTN_REPEAT) {
    irrecv.resume();
    return;
  }

  if (code == BTN_1) {
    routeToIntersection3();
  }
  else if (code == BTN_2) {
    routeToIntersection3AndTurnRight();
  }
  else if (code == BTN_3) {
    routeToIntersection3TurnRightAndStrafeLeft();
  }
  else if (code == BTN_4) {
    routeTimedForwardTurnRightAndStrafeLeft();
  }
  else if (code == BTN_5) {
    routeLeftAt2RightAt3();
  }
  else if (code == BTN_6) {
    routeLeftAt2RightAt3ThenStrafeLeft();
  }
  else if (code == BTN_7) {
    routeButton6AndReturnToStart();
  }
  else if (code == BTN_LEFT) {
    manualStrafeLeft();
  }
  else if (code == BTN_0 || code == BTN_STAR) {
    manualStop();
  }

  irrecv.resume();
}
