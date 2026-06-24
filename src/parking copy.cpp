/*
  ================================================================
  KS0560 Mecanum Car - LINE FOLLOW TO INTERSECTION 3
  ================================================================
  LOGIC:
    - Button 1: Follow the straight line and stop at intersection 3.
    - Button 2: Follow the straight line, stop at intersection 3, then turn right 90 degrees.
    - Button 3: Follow the straight line, stop at intersection 3, turn right 90 degrees, then strafe left.
    - Button 4: Drive forward by time, turn right by time, then strafe left by time.
    - Button 5: Turn left at intersection 2, right at intersection 3, then go 3 more intersections and turn 180 degrees.
    - Button 6: Go straight 2 intersections, strafe left 2 middle-sensor lines, then go straight 2 intersections.
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
#define BTN_STAR        0xFF42BD
#define BTN_REPEAT      0xFFFFFFFF

// Speeds
#define SPEED_FORWARD   50  // 60
#define SPEED_TURN      45
#define SPEED_ROTATE    60  //60
#define SPEED_STRAFE    60  //55
#define SPEED_MIN       35

// Line configuration
#define BLACK_LINE      HIGH
#define TARGET_BLOCKS   3
#define STOP_SENSOR_RIGHT_TURN SENSOR_RIGHT
#define STOP_SENSOR_LEFT_TURN SENSOR_LEFT
#define TURN_BLIND_MS   150
#define STRAFE_LEFT_MS  2000
#define TARGET_STRAFE_LINES 2
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

// ================================================================
//  FOLLOW LINE AND STOP AT INTERSECTION 3
// ================================================================
bool moveForwardBlocks(int targetBlocks, uint8_t startSpeed) {
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

  car.Stop();
  delay(200);
  Serial.println(F("--- Movement Completed. ---"));
  restoreIR();
  return completed;
}

// ================================================================
//  TURN 90 DEGREES RIGHT
// ================================================================
bool rotateRight90() {
  Serial.println(F("\n--- Rotating 90 Degrees Right ---"));

  unsigned long startTime = millis();
  while (millis() - startTime < TURN_BLIND_MS) {
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

    if (digitalRead(STOP_SENSOR_RIGHT_TURN) == BLACK_LINE) {
      break;
    }
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- Right Turn Completed. ---"));
  restoreIR();
  return true;
}

// ================================================================
//  TURN 90 DEGREES LEFT
// ================================================================
bool rotateLeft90() {
  Serial.println(F("\n--- Rotating 90 Degrees Left ---"));

  unsigned long startTime = millis();
  while (millis() - startTime < TURN_BLIND_MS) {
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

    if (digitalRead(STOP_SENSOR_LEFT_TURN) == BLACK_LINE) {
      break;
    }
  }

  car.Stop();
  delay(200);
  Serial.println(F("--- Left Turn Completed. ---"));
  restoreIR();
  return true;
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

// ================================================================
//  STRAFE LEFT AND COUNT LINES WITH MIDDLE SENSOR ONLY
// ================================================================
bool strafeLeftLines(int targetLines) {
  int lineCount = 0;
  bool onLine = digitalRead(SENSOR_MID) == BLACK_LINE;
  bool completed = false;

  Serial.print(F("\n--- Strafing Left to Cross "));
  Serial.print(targetLines);
  Serial.println(F(" Lines with Middle Sensor ---"));

  while (lineCount < targetLines) {
    if (emergencyStopPressed()) {
      break;
    }

    uint8_t M = digitalRead(SENSOR_MID);

    setMotorSpeed(SPEED_STRAFE);
    car.L_Move();

    if (M == BLACK_LINE) {
      if (!onLine) {
        lineCount++;
        onLine = true;

        Serial.print(F("Strafe line: "));
        Serial.println(lineCount);

        if (lineCount >= targetLines) {
          completed = true;
          break;
        }
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
  moveForwardBlocks(TARGET_BLOCKS, SPEED_FORWARD);
}

bool routeToIntersection3AndTurnRight() {
  if (!moveForwardBlocks(TARGET_BLOCKS, SPEED_FORWARD)) {
    return false;
  }

  return rotateRight90();
}

void routeToIntersection3TurnRightAndStrafeLeft() {
  if (routeToIntersection3AndTurnRight()) {
    strafeLeftFor(STRAFE_LEFT_MS);
  }
}

void routeTimedForwardTurnRightAndStrafeLeft() {
  if (driveStraightFor(TIMED_FORWARD_MS) && rotateRightFor(TIMED_RIGHT_TURN_MS)) {
    strafeLeftFor(STRAFE_LEFT_MS);
  }
}

void routeLeftAt2RightAt3() {
  if (!moveForwardBlocks(2, SPEED_FORWARD)) {
    return;
  }

  if (!rotateLeft90()) {
    return;
  }

  if (!moveForwardBlocks(3, SPEED_FORWARD)) {
    return;
  }

  if (!rotateRight90()) {
    return;
  }

  if (moveForwardBlocks(3, SPEED_FORWARD)) {
    rotateRight180();
  }
}

void routeLeftAt2RightAt3ThenStrafeLeft() {
  if (!moveForwardBlocks(2, SPEED_FORWARD)) {
    return;
  }

  if (!strafeLeftLines(TARGET_STRAFE_LINES)) {
    return;
  }

  moveForwardBlocks(2, SPEED_FORWARD);
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
  Serial.println(F("Press '5' to turn left at intersection 2, right at intersection 3, then turn 180 after 3 more intersections."));
  Serial.println(F("Press '6' to go straight 2 intersections, strafe 2 middle-sensor lines, then go straight 2 intersections."));
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

  irrecv.resume();
}
