#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.h>
#include <Servo.h>

// IR Hex Codes
#define BTN_1      0xFF6897 // Starts Line Tracking -> 3 intersections -> turn right -> strafe left 2 lines
#define BTN_2      0xFF9867
#define BTN_3      0xFFB04F
#define BTN_4      0xFF30CF
#define BTN_5      0xFF18E7
#define BTN_6      0xFF7A85
#define BTN_UP     0xFF629D
#define BTN_DOWN   0xFFA857
#define BTN_LEFT   0xFF22DD
#define BTN_RIGHT  0xFFC23D
#define BTN_0      0xFF4AB5
#define BTN_OK     0xFF02FD

// Hardware Pins
#define IR_PIN A3
#define EchoPin 13
#define TrigPin 12
#define SensorLeft   A0
#define SensorMiddle A1
#define SensorRight  A2
#define GripperPin   9

// TCS3200 Color Sensor Pins
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

// Configuration
#define SPEED 60
#define TURN_SPEED 35
#define MICRO_TURN_SPEED 35
#define STOP_DISTANCE 32.0

#define BLACK_LINE HIGH

// Dock/action tuning
#define TARGET_FORWARD_INTERSECTIONS 3
#define TARGET_STRAFE_LINES 2
#define RIGHT_TURN_TIME 520
#define TURN_IGNORE_TIME 120
#define STRAFE_SPEED 56

mecanumCar car(3, 2);
IRrecv irrecv(IR_PIN);
decode_results results;
Servo gripper;

enum RobotState {
  STATE_INIT,
  IDLE, MOVE_1, MOVE_2, MOVE_3, MOVE_4, MOVE_5, MOVE_6,
  MANUAL_FWD, MANUAL_BWD, MANUAL_LEFT, MANUAL_RIGHT
};

enum ColorDetected { NONE, RED, GREEN, BLUE };

enum AutoPhase {
  AUTO_IDLE,
  AUTO_LINE_TRACK_COUNT,
  AUTO_TURN_RIGHT,
  AUTO_STRAFE_LEFT_COUNT,
  AUTO_DONE
};

RobotState currentState = STATE_INIT;
RobotState lastState = STATE_INIT;
AutoPhase autoPhase = AUTO_IDLE;

bool lightsOn = false;
unsigned long lastSonarTime = 0;
unsigned long lastColorTime = 0;
unsigned long lastSerialTime = 0;
unsigned long phaseStartTime = 0;

// Forward intersection tracking
int intersectionCount = 0;
bool isOnIntersection = false;

// Strafe line tracking
int strafeLineCount = 0;
bool isOnStrafeLine = false;

ColorDetected currentDetectedColor = NONE;

int rawR = 0;
int rawG = 0;
int rawB = 0;

// --- Helper Functions ---

void PrintLineCounters() {
  Serial.print("Forward lines: ");
  Serial.print(intersectionCount);
  Serial.print(" | Strafe lines: ");
  Serial.println(strafeLineCount);
}

float Get_Distance(void) {
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  float dis = pulseIn(EchoPin, HIGH, 30000) / 58.2;

  if (dis == 0 || dis < 10.0) return 999.0;
  return dis;
}

ColorDetected IdentifyColor() {
  car.right_led(0);
  car.left_led(0);
  delay(10);

  digitalWrite(S2, LOW); digitalWrite(S3, LOW); delay(15);
  rawR = pulseIn(sensorOut, LOW, 20000);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); delay(15);
  rawG = pulseIn(sensorOut, LOW, 20000);

  digitalWrite(S2, LOW); digitalWrite(S3, HIGH); delay(15);
  rawB = pulseIn(sensorOut, LOW, 20000);

  if (rawR == 0 || rawG == 0 || rawB == 0) return NONE;
  if (rawR > 350 && rawG > 350 && rawB > 350) return NONE;

  if (rawR < (rawG - 15) && rawR < (rawB - 15)) return RED;
  else if (rawG < (rawR - 15) && rawG < (rawB - 15)) return GREEN;
  else if (rawB < (rawR - 15) && rawB < (rawG - 15)) return BLUE;

  return NONE;
}

void PrintFSMState(RobotState state) {
  (void)state;
}

void PrintAutoPhase() {
  PrintLineCounters();
}

void ResetAutoSequence() {
  autoPhase = AUTO_IDLE;
  phaseStartTime = 0;
  intersectionCount = 0;
  isOnIntersection = false;
  strafeLineCount = 0;
  isOnStrafeLine = false;
}

void MoveForwardRaw(int leftSpd, int rightSpd) {
  car.Motor_Upper_L(1, leftSpd);  car.Motor_Upper_R(1, rightSpd);
  car.Motor_Lower_L(1, leftSpd);  car.Motor_Lower_R(1, rightSpd);
}

void RotateRightRaw(int spd) {
  car.Motor_Upper_L(1, spd); car.Motor_Upper_R(0, spd);
  car.Motor_Lower_L(1, spd); car.Motor_Lower_R(0, spd);
}

bool IsIntersection(uint8_t SL, uint8_t SM, uint8_t SR) {
  return (SL == BLACK_LINE && SM == BLACK_LINE && SR == BLACK_LINE);
}

bool IsAnyBlack(uint8_t SL, uint8_t SM, uint8_t SR) {
  return (SL == BLACK_LINE || SM == BLACK_LINE || SR == BLACK_LINE);
}

void RunForwardLineTrackingAndCount() {
  uint8_t SL = digitalRead(SensorLeft);
  uint8_t SM = digitalRead(SensorMiddle);
  uint8_t SR = digitalRead(SensorRight);

  if (IsIntersection(SL, SM, SR)) {
    MoveForwardRaw(SPEED, SPEED);

    if (!isOnIntersection) {
      intersectionCount++;
      isOnIntersection = true;
      PrintLineCounters();

      if (intersectionCount >= TARGET_FORWARD_INTERSECTIONS) {
        car.Stop();
        delay(200);
        autoPhase = AUTO_TURN_RIGHT;
        phaseStartTime = millis();
        PrintAutoPhase();
      }
    }
    return;
  } else {
    isOnIntersection = false;
  }

  if (SL != BLACK_LINE && SM == BLACK_LINE && SR != BLACK_LINE) {
    MoveForwardRaw(SPEED, SPEED);
  }
  else if (SL == BLACK_LINE && SM == BLACK_LINE && SR != BLACK_LINE) {
    MoveForwardRaw(MICRO_TURN_SPEED, SPEED);
  }
  else if (SL != BLACK_LINE && SM == BLACK_LINE && SR == BLACK_LINE) {
    MoveForwardRaw(SPEED, MICRO_TURN_SPEED);
  }
  else if (SL == BLACK_LINE && SM != BLACK_LINE && SR != BLACK_LINE) {
    car.Motor_Upper_L(1, 0);          car.Motor_Upper_R(1, TURN_SPEED);
    car.Motor_Lower_L(1, 0);          car.Motor_Lower_R(1, TURN_SPEED);
  }
  else if (SL != BLACK_LINE && SM != BLACK_LINE && SR == BLACK_LINE) {
    car.Motor_Upper_L(1, TURN_SPEED); car.Motor_Upper_R(1, 0);
    car.Motor_Lower_L(1, TURN_SPEED); car.Motor_Lower_R(1, 0);
  }
  else {
    MoveForwardRaw(SPEED, SPEED);
  }
}

void RunStrafeLeftAndCountLines() {
  uint8_t SL = digitalRead(SensorLeft);
  uint8_t SM = digitalRead(SensorMiddle);
  uint8_t SR = digitalRead(SensorRight);

  car.L_Move();

  // Count line crossing on rising edge only
  if (IsAnyBlack(SL, SM, SR)) {
    if (!isOnStrafeLine) {
      isOnStrafeLine = true;
      strafeLineCount++;
      PrintLineCounters();

      if (strafeLineCount >= TARGET_STRAFE_LINES) {
        car.Stop();
        autoPhase = AUTO_DONE;
        PrintAutoPhase();
      }
    }
  } else {
    isOnStrafeLine = false;
  }
}

void RunAutoRoute(unsigned long currentMillis) {
  switch (autoPhase) {
    case AUTO_IDLE:
      autoPhase = AUTO_LINE_TRACK_COUNT;
      PrintAutoPhase();
      break;

    case AUTO_LINE_TRACK_COUNT:
      RunForwardLineTrackingAndCount();
      break;

    case AUTO_TURN_RIGHT:
      RotateRightRaw(TURN_SPEED);
      if (currentMillis - phaseStartTime >= RIGHT_TURN_TIME) {
        car.Stop();
        autoPhase = AUTO_STRAFE_LEFT_COUNT;
        phaseStartTime = currentMillis;
        isOnStrafeLine = false;
        PrintAutoPhase();
      }
      break;

    case AUTO_STRAFE_LEFT_COUNT:
      // Small sensor ignore window at start of strafe to avoid false trigger from turn exit
      if (currentMillis - phaseStartTime < TURN_IGNORE_TIME) {
        car.L_Move();
      } else {
        RunStrafeLeftAndCountLines();
      }
      break;

    case AUTO_DONE:
      car.Stop();
      currentState = IDLE;
      break;
  }
}

// --- Setup & Audit ---

void setup() {
  Serial.begin(9600);

  car.Init();
  gripper.attach(GripperPin);

  pinMode(EchoPin, INPUT);
  pinMode(TrigPin, OUTPUT);

  pinMode(SensorLeft, INPUT);
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight, INPUT);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH); digitalWrite(S1, LOW);
}

void ExecuteSelfAudit() {
  delay(500);

  float dist = Get_Distance();
  (void)dist;

  digitalWrite(S2, LOW); digitalWrite(S3, LOW); delay(15);
  int testR = pulseIn(sensorOut, LOW, 20000);
  (void)testR;

  car.right_led(1); car.left_led(1); delay(300);
  car.right_led(0); car.left_led(0); delay(100);

  car.Motor_Upper_L(1, 60); car.Motor_Upper_R(1, 60);
  car.Motor_Lower_L(1, 60); car.Motor_Lower_R(1, 60);
  delay(60);
  car.Back();
  delay(60);
  car.Stop();

  irrecv.enableIRIn();
  while (irrecv.decode(&results)) {
    irrecv.resume();
  }

  gripper.write(60); delay(300);
  gripper.write(90); delay(300);

  PrintLineCounters();
}

// --- Main Loop ---

void loop() {
  unsigned long currentMillis = millis();

  // 0. Boot Sequence
  if (currentState == STATE_INIT) {
    ExecuteSelfAudit();
    currentState = IDLE;
  }

  // 1. Live Collision Check
  if (currentState == MANUAL_FWD || currentState == MOVE_1) {
    if (currentMillis - lastSonarTime >= 50) {
      if (Get_Distance() <= STOP_DISTANCE) {
        currentState = IDLE;
        ResetAutoSequence();
      }
      lastSonarTime = currentMillis;
    }
  }

  // 2. Color Detection & LED Patterns
  if (currentMillis - lastColorTime >= 250) {
    currentDetectedColor = IdentifyColor();

    if (lightsOn) {
      car.right_led(1); car.left_led(1);
    }
    else {
      switch(currentDetectedColor) {
        case NONE:  car.right_led(0); car.left_led(0); break;
        case GREEN: car.right_led(1); car.left_led(1); break;
        case RED:
          if ((currentMillis / 100) % 2 == 0) { car.right_led(1); car.left_led(1); }
          else { car.right_led(0); car.left_led(0); }
          break;
        case BLUE:
          if ((currentMillis / 500) % 2 == 0) { car.right_led(1); car.left_led(1); }
          else { car.right_led(0); car.left_led(0); }
          break;
      }
    }
    lastColorTime = currentMillis;
  }

  // 3. Serial Monitor Output
  if (currentMillis - lastSerialTime >= 1000) {
    PrintLineCounters();
    lastSerialTime = currentMillis;
  }

  // 4. Digital Perception Block (IR)
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;

    if (code == BTN_OK) { lightsOn = !lightsOn; delay(300); }
    else if (code == BTN_0) { currentState = IDLE; ResetAutoSequence(); delay(300); }
    else if (code == BTN_UP) { currentState = MANUAL_FWD; ResetAutoSequence(); delay(300); }
    else if (code == BTN_DOWN) { currentState = MANUAL_BWD; ResetAutoSequence(); delay(300); }
    else if (code == BTN_LEFT) { currentState = MANUAL_LEFT; ResetAutoSequence(); delay(300); }
    else if (code == BTN_RIGHT) { currentState = MANUAL_RIGHT; ResetAutoSequence(); delay(300); }
    else if (code == BTN_1) {
      ResetAutoSequence();
      currentState = MOVE_1;
      delay(300);
    }
    else if (code == BTN_2) { currentState = MOVE_2; delay(300); }
    else if (code == BTN_3) { currentState = MOVE_3; delay(300); }
    else if (code == BTN_4) { currentState = MOVE_4; delay(300); }
    else if (code == BTN_5) { currentState = MOVE_5; delay(300); }
    else if (code == BTN_6) { currentState = MOVE_6; delay(300); }

    irrecv.resume();
  }

  // 5. Dynamic Continuous Logic
  if (currentState == MOVE_1) {
    RunAutoRoute(currentMillis);
  }

  // 6. FSM Monitor & Static Logic Execution
  if (currentState != lastState) {
    if (currentState != STATE_INIT) {
      PrintFSMState(currentState);
    }

    switch(currentState) {
      case IDLE: car.Stop(); break;
      case MANUAL_FWD:
        car.Motor_Upper_L(1, SPEED); car.Motor_Upper_R(1, SPEED);
        car.Motor_Lower_L(1, SPEED); car.Motor_Lower_R(1, SPEED); break;
      case MANUAL_BWD: car.Back(); break;
      case MANUAL_LEFT: car.L_Move(); break;
      case MANUAL_RIGHT: car.R_Move(); break;
      case MOVE_1:
        break;
      case MOVE_2:
        car.Motor_Upper_L(1, SPEED); car.Motor_Upper_R(0, SPEED);
        car.Motor_Lower_L(0, SPEED); car.Motor_Lower_R(1, SPEED); break;
      case MOVE_3:
        car.Motor_Upper_L(1, SPEED); car.Motor_Lower_R(1, SPEED);
        car.Motor_Upper_R(0, 0);     car.Motor_Lower_L(0, 0); break;
      case MOVE_4:
        car.Motor_Upper_L(1, SPEED); car.Motor_Lower_L(1, SPEED);
        car.Motor_Upper_R(0, 0);     car.Motor_Lower_R(0, 0); break;
      case MOVE_5:
        car.Motor_Upper_L(1, SPEED); car.Motor_Lower_L(1, SPEED);
        car.Motor_Upper_R(0, SPEED); car.Motor_Lower_R(0, SPEED); break;
      case MOVE_6:
        car.Motor_Upper_L(1, SPEED); car.Motor_Upper_R(0, SPEED);
        car.Motor_Lower_L(0, 0);     car.Motor_Lower_R(0, 0); break;
    }
    lastState = currentState;
  }
}
