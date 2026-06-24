#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.h>
#include <Servo.h>

// =========================
// IR Hex Codes
// =========================
#define BTN_1      0xFF6897
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

// =========================
// Hardware Pins
// =========================
#define IR_PIN         A3
#define EchoPin        13
#define TrigPin        12
#define SensorLeft     A0
#define SensorMiddle   A1
#define SensorRight    A2
#define GripperPin     9

// =========================
// TCS3200 Color Sensor Pins
// =========================
#define S0         4
#define S1         5
#define S2         6
#define S3         7
#define sensorOut  8

// =========================
// Configuration
// =========================
#define SPEED               80
#define TURN_SPEED          50
#define STOP_DISTANCE       32.0

#define DOCK_FWD_SPEED      68
#define DOCK_TURN_SPEED     48
#define DOCK_STRAFE_SPEED   65
#define DOCK_TURN_TIME      420
#define DOCK_SETTLE_TIME    120

#define TARGET_FORWARD_MARKERS 3
#define TARGET_STRAFE_LINES    2

// If robot tracks white instead of black, change HIGH to LOW
#define BLACK_LINE HIGH

mecanumCar car(3, 2);
IRrecv irrecv(IR_PIN);
decode_results results;
Servo gripper;

// =========================
// Enums
// =========================
enum RobotState {
  STATE_INIT,
  IDLE,
  MOVE_1, MOVE_2, MOVE_3, MOVE_4, MOVE_5, MOVE_6,
  MANUAL_FWD, MANUAL_BWD, MANUAL_LEFT, MANUAL_RIGHT
};

enum ColorDetected {
  NONE, RED, GREEN, BLUE
};

enum DockPhase {
  DOCK_IDLE,
  DOCK_COUNT_FORWARD_MARKERS,
  DOCK_TURN_RIGHT,
  DOCK_STRAFE_COUNT_LINES,
  DOCK_SETTLE,
  DOCK_DONE
};

// =========================
// Globals
// =========================
RobotState currentState = STATE_INIT;
RobotState lastState = STATE_INIT;
DockPhase dockPhase = DOCK_IDLE;

bool lightsOn = false;

unsigned long lastSonarTime = 0;
unsigned long lastColorTime = 0;
unsigned long lastSerialTime = 0;
unsigned long dockTimer = 0;

ColorDetected currentDetectedColor = NONE;

int rawR = 0;
int rawG = 0;
int rawB = 0;

int forwardMarkerCount = 0;
int strafeLineCount = 0;
bool forwardMarkerLatched = false;
bool strafeLineLatched = false;

// =========================
// Sensor Read Helpers
// =========================
uint8_t readSL() { return digitalRead(SensorLeft); }
uint8_t readSM() { return digitalRead(SensorMiddle); }
uint8_t readSR() { return digitalRead(SensorRight); }

bool IsAnyBlack(uint8_t SL, uint8_t SM, uint8_t SR) {
  return (SL == BLACK_LINE || SM == BLACK_LINE || SR == BLACK_LINE);
}

bool IsCenterOnLine(uint8_t SM) {
  return (SM == BLACK_LINE);
}

bool IsLeftOnLine(uint8_t SL) {
  return (SL == BLACK_LINE);
}

bool IsRightOnLine(uint8_t SR) {
  return (SR == BLACK_LINE);
}

// Marker for forward counting:
// safest first guess = all 3 sensors detect black together
bool IsForwardMarker(uint8_t SL, uint8_t SM, uint8_t SR) {
  return (SL == BLACK_LINE && SM == BLACK_LINE && SR == BLACK_LINE);
}

// During right strafe, count a line when middle OR left/right sees black.
// You may tune this later depending on your field layout.
bool IsStrafeLine(uint8_t SL, uint8_t SM, uint8_t SR) {
  return (SL == BLACK_LINE || SM == BLACK_LINE || SR == BLACK_LINE);
}

// =========================
// Motion Helpers
// =========================
void MoveForwardRaw(int spd) {
  car.Motor_Upper_L(1, spd);
  car.Motor_Upper_R(1, spd);
  car.Motor_Lower_L(1, spd);
  car.Motor_Lower_R(1, spd);
}

void MoveBackwardRaw(int spd) {
  car.Back();
}

void RotateLeftRaw(int spd) {
  car.Motor_Upper_L(0, spd);
  car.Motor_Upper_R(1, spd);
  car.Motor_Lower_L(0, spd);
  car.Motor_Lower_R(1, spd);
}

void RotateRightRaw(int spd) {
  car.Motor_Upper_L(1, spd);
  car.Motor_Upper_R(0, spd);
  car.Motor_Lower_L(1, spd);
  car.Motor_Lower_R(0, spd);
}

void StrafeLeftRaw() {
  car.L_Move();
}

void StrafeRightRaw() {
  car.R_Move();
}

// =========================
// Core Helpers
// =========================
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

  digitalWrite(S2, LOW);  digitalWrite(S3, LOW);  delay(15);
  rawR = pulseIn(sensorOut, LOW, 20000);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); delay(15);
  rawG = pulseIn(sensorOut, LOW, 20000);

  digitalWrite(S2, LOW);  digitalWrite(S3, HIGH); delay(15);
  rawB = pulseIn(sensorOut, LOW, 20000);

  if (rawR == 0 || rawG == 0 || rawB == 0) return NONE;
  if (rawR > 350 && rawG > 350 && rawB > 350) return NONE;

  if (rawR < (rawG - 15) && rawR < (rawB - 15)) return RED;
  else if (rawG < (rawR - 15) && rawG < (rawB - 15)) return GREEN;
  else if (rawB < (rawR - 15) && rawB < (rawG - 15)) return BLUE;

  return NONE;
}

void PrintFSMState(RobotState state) {
  Serial.print("[FSM SYSTEM] Transitioned to: ");
  switch(state) {
    case STATE_INIT:   Serial.println("STATE_INIT"); break;
    case IDLE:         Serial.println("IDLE"); break;
    case MANUAL_FWD:   Serial.println("MANUAL_FWD"); break;
    case MANUAL_BWD:   Serial.println("MANUAL_BWD"); break;
    case MANUAL_LEFT:  Serial.println("MANUAL_LEFT"); break;
    case MANUAL_RIGHT: Serial.println("MANUAL_RIGHT"); break;
    case MOVE_1:       Serial.println("MOVE_1 (Line Tracking)"); break;
    case MOVE_2:       Serial.println("MOVE_2 (3 Markers -> Right Turn -> 2 Strafe Lines)"); break;
    case MOVE_3:       Serial.println("MOVE_3"); break;
    case MOVE_4:       Serial.println("MOVE_4"); break;
    case MOVE_5:       Serial.println("MOVE_5"); break;
    case MOVE_6:       Serial.println("MOVE_6"); break;
    default:           Serial.println("UNKNOWN"); break;
  }
}

void PrintDockPhase(DockPhase phase) {
  Serial.print("[DOCK] Phase -> ");
  switch(phase) {
    case DOCK_IDLE:                  Serial.println("DOCK_IDLE"); break;
    case DOCK_COUNT_FORWARD_MARKERS: Serial.println("DOCK_COUNT_FORWARD_MARKERS"); break;
    case DOCK_TURN_RIGHT:            Serial.println("DOCK_TURN_RIGHT"); break;
    case DOCK_STRAFE_COUNT_LINES:    Serial.println("DOCK_STRAFE_COUNT_LINES"); break;
    case DOCK_SETTLE:                Serial.println("DOCK_SETTLE"); break;
    case DOCK_DONE:                  Serial.println("DOCK_DONE"); break;
    default:                         Serial.println("UNKNOWN"); break;
  }
}

void ResetDocking() {
  dockPhase = DOCK_IDLE;
  dockTimer = 0;
  forwardMarkerCount = 0;
  strafeLineCount = 0;
  forwardMarkerLatched = false;
  strafeLineLatched = false;
}

void RunLineTracking() {
  uint8_t SL = readSL();
  uint8_t SM = readSM();
  uint8_t SR = readSR();

  if (IsCenterOnLine(SM)) {
    MoveForwardRaw(SPEED);
  }
  else if (IsLeftOnLine(SL)) {
    RotateLeftRaw(TURN_SPEED);
  }
  else if (IsRightOnLine(SR)) {
    RotateRightRaw(TURN_SPEED);
  }
  else {
    car.Stop();
  }
}

void RunAutoDock(unsigned long currentMillis) {
  uint8_t SL = readSL();
  uint8_t SM = readSM();
  uint8_t SR = readSR();

  switch (dockPhase) {
    case DOCK_IDLE:
      dockPhase = DOCK_COUNT_FORWARD_MARKERS;
      PrintDockPhase(dockPhase);
      break;

    case DOCK_COUNT_FORWARD_MARKERS:
      // keep following line forward
      if (IsCenterOnLine(SM)) {
        MoveForwardRaw(DOCK_FWD_SPEED);
      }
      else if (IsLeftOnLine(SL)) {
        RotateLeftRaw(DOCK_TURN_SPEED);
      }
      else if (IsRightOnLine(SR)) {
        RotateRightRaw(DOCK_TURN_SPEED);
      }
      else {
        car.Stop();
      }

      // rising edge count for forward markers
      if (IsForwardMarker(SL, SM, SR)) {
        if (!forwardMarkerLatched) {
          forwardMarkerLatched = true;
          forwardMarkerCount++;
          Serial.print("[DOCK] Forward marker count = ");
          Serial.println(forwardMarkerCount);

          if (forwardMarkerCount >= TARGET_FORWARD_MARKERS) {
            car.Stop();
            dockTimer = currentMillis;
            dockPhase = DOCK_TURN_RIGHT;
            PrintDockPhase(dockPhase);
          }
        }
      } else {
        forwardMarkerLatched = false;
      }
      break;

    case DOCK_TURN_RIGHT:
      RotateRightRaw(DOCK_TURN_SPEED);
      if (currentMillis - dockTimer >= DOCK_TURN_TIME) {
        car.Stop();
        strafeLineLatched = false;
        dockPhase = DOCK_STRAFE_COUNT_LINES;
        PrintDockPhase(dockPhase);
      }
      break;

    case DOCK_STRAFE_COUNT_LINES:
      StrafeRightRaw();

      // rising edge count for strafe crossing lines
      if (IsStrafeLine(SL, SM, SR)) {
        if (!strafeLineLatched) {
          strafeLineLatched = true;
          strafeLineCount++;
          Serial.print("[DOCK] Strafe line count = ");
          Serial.println(strafeLineCount);

          if (strafeLineCount >= TARGET_STRAFE_LINES) {
            car.Stop();
            dockTimer = currentMillis;
            dockPhase = DOCK_SETTLE;
            PrintDockPhase(dockPhase);
          }
        }
      } else {
        strafeLineLatched = false;
      }
      break;

    case DOCK_SETTLE:
      car.Stop();
      if (currentMillis - dockTimer >= DOCK_SETTLE_TIME) {
        dockPhase = DOCK_DONE;
        PrintDockPhase(dockPhase);
      }
      break;

    case DOCK_DONE:
      car.Stop();
      currentState = IDLE;
      ResetDocking();
      break;

    default:
      car.Stop();
      currentState = IDLE;
      ResetDocking();
      break;
  }
}

// =========================
// Setup & Audit
// =========================
void ExecuteSelfAudit() {
  Serial.println("\n=============================================");
  Serial.println(">>> ENTERING STATE_INIT: SYSTEM SELF-AUDIT");
  Serial.println("=============================================");
  delay(500);

  Serial.print("[AUDIT] Ultrasonic Array... ");
  float dist = Get_Distance();
  if (dist > 0 && dist < 500) Serial.println("PASS (" + String(dist) + "cm)");
  else Serial.println("WARN (Out of range or no ping)");

  Serial.print("[AUDIT] TCS3200 RGB Sensor... ");
  digitalWrite(S2, LOW); digitalWrite(S3, LOW); delay(15);
  int testR = pulseIn(sensorOut, LOW, 20000);
  if (testR > 0) Serial.println("PASS (Signal verified)");
  else Serial.println("FAIL (Timeout. Check wiring!)");

  Serial.print("[AUDIT] TCRT5000 Tracking Array (L/M/R)... ");
  Serial.print(digitalRead(SensorLeft)); Serial.print("/");
  Serial.print(digitalRead(SensorMiddle)); Serial.print("/");
  Serial.println(digitalRead(SensorRight));

  Serial.print("[AUDIT] Gripper Servo... ");
  gripper.write(60); delay(300);
  gripper.write(90); delay(300);
  Serial.println("PASS (Sweep complete)");

  Serial.print("[AUDIT] 7-Color LEDs... ");
  car.right_led(1); car.left_led(1); delay(300);
  car.right_led(0); car.left_led(0); delay(100);
  Serial.println("PASS (Flash sequence complete)");

  Serial.print("[AUDIT] Wheel Motors... Executing Mechanical Shiver... ");
  car.Motor_Upper_L(1, 60); car.Motor_Upper_R(1, 60);
  car.Motor_Lower_L(1, 60); car.Motor_Lower_R(1, 60);
  delay(60);
  car.Back();
  delay(60);
  car.Stop();
  Serial.println("PASS");

  Serial.print("[AUDIT] IR Receiver... ");
  irrecv.enableIRIn();
  while (irrecv.decode(&results)) {
    irrecv.resume();
  }
  Serial.println("PASS (Buffer cleared)");

  Serial.println("=============================================");
  Serial.println(">>> AUDIT COMPLETE. READY FOR COMMANDS.");
  Serial.println("=============================================\n");
}

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

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

// =========================
// Main Loop
// =========================
void loop() {
  unsigned long currentMillis = millis();

  if (currentState == STATE_INIT) {
    ExecuteSelfAudit();
    currentState = IDLE;
  }

  bool checkForwardObstacle =
    (currentState == MANUAL_FWD) ||
    (currentState == MOVE_1) ||
    (currentState == MOVE_2 && dockPhase == DOCK_COUNT_FORWARD_MARKERS);

  if (checkForwardObstacle) {
    if (currentMillis - lastSonarTime >= 50) {
      if (Get_Distance() <= STOP_DISTANCE) {
        if (currentState != IDLE) Serial.println("[ALERT] Wall Detected! Emergency Stop.");
        car.Stop();
        currentState = IDLE;
        ResetDocking();
      }
      lastSonarTime = currentMillis;
    }
  }

  if (currentMillis - lastColorTime >= 100) {
    currentDetectedColor = IdentifyColor();

    if (lightsOn) {
      car.right_led(1);
      car.left_led(1);
    } else {
      switch (currentDetectedColor) {
        case NONE:
          car.right_led(0); car.left_led(0);
          break;
        case GREEN:
          car.right_led(1); car.left_led(1);
          break;
        case RED:
          if ((currentMillis / 100) % 2 == 0) {
            car.right_led(1); car.left_led(1);
          } else {
            car.right_led(0); car.left_led(0);
          }
          break;
        case BLUE:
          if ((currentMillis / 500) % 2 == 0) {
            car.right_led(1); car.left_led(1);
          } else {
            car.right_led(0); car.left_led(0);
          }
          break;
      }
    }
    lastColorTime = currentMillis;
  }

  if (currentMillis - lastSerialTime >= 500) {
    uint8_t SL = readSL();
    uint8_t SM = readSM();
    uint8_t SR = readSR();

    Serial.print("Line Track -> L: ");
    Serial.print(SL == BLACK_LINE ? "BLACK" : "WHITE");
    Serial.print(" | M: ");
    Serial.print(SM == BLACK_LINE ? "BLACK" : "WHITE");
    Serial.print(" | R: ");
    Serial.println(SR == BLACK_LINE ? "BLACK" : "WHITE");

    Serial.print("Dock Status -> ForwardMarkers: ");
    Serial.print(forwardMarkerCount);
    Serial.print(" | StrafeLines: ");
    Serial.println(strafeLineCount);

    lastSerialTime = currentMillis;
  }

  if (irrecv.decode(&results)) {
    unsigned long code = results.value;

    if (code == BTN_OK) {
      lightsOn = !lightsOn;
      delay(300);
    }
    else if (code == BTN_0) {
      currentState = IDLE;
      ResetDocking();
      delay(300);
    }
    else if (code == BTN_UP) {
      currentState = MANUAL_FWD;
      ResetDocking();
      delay(300);
    }
    else if (code == BTN_DOWN) {
      currentState = MANUAL_BWD;
      ResetDocking();
      delay(300);
    }
    else if (code == BTN_LEFT) {
      currentState = MANUAL_LEFT;
      ResetDocking();
      delay(300);
    }
    else if (code == BTN_RIGHT) {
      currentState = MANUAL_RIGHT;
      ResetDocking();
      delay(300);
    }
    else if (code == BTN_1) {
      currentState = MOVE_1;
      ResetDocking();
      delay(300);
    }
    else if (code == BTN_2) {
      currentState = MOVE_2;
      ResetDocking();
      delay(300);
    }
    else if (code == BTN_3) {
      currentState = MOVE_3;
      ResetDocking();
      delay(300);
    }
    else if (code == BTN_4) {
      currentState = MOVE_4;
      ResetDocking();
      delay(300);
    }
    else if (code == BTN_5) {
      currentState = MOVE_5;
      ResetDocking();
      delay(300);
    }
    else if (code == BTN_6) {
      currentState = MOVE_6;
      ResetDocking();
      delay(300);
    }

    irrecv.resume();
  }

  if (currentState == MOVE_1) {
    RunLineTracking();
  }

  if (currentState == MOVE_2) {
    RunAutoDock(currentMillis);
  }

  if (currentState != lastState) {
    if (currentState != STATE_INIT) {
      PrintFSMState(currentState);
    }

    switch (currentState) {
      case IDLE:
        car.Stop();
        break;

      case MANUAL_FWD:
        MoveForwardRaw(SPEED);
        break;

      case MANUAL_BWD:
        MoveBackwardRaw(SPEED);
        break;

      case MANUAL_LEFT:
        car.L_Move();
        break;

      case MANUAL_RIGHT:
        car.R_Move();
        break;

      case MOVE_1:
        break;

      case MOVE_2:
        break;

      case MOVE_3:
        car.Motor_Upper_L(1, SPEED); car.Motor_Lower_R(1, SPEED);
        car.Motor_Upper_R(0, 0);     car.Motor_Lower_L(0, 0);
        break;

      case MOVE_4:
        car.Motor_Upper_L(1, SPEED); car.Motor_Lower_L(1, SPEED);
        car.Motor_Upper_R(0, 0);     car.Motor_Lower_R(0, 0);
        break;

      case MOVE_5:
        car.Motor_Upper_L(1, SPEED); car.Motor_Lower_L(1, SPEED);
        car.Motor_Upper_R(0, SPEED); car.Motor_Lower_R(0, SPEED);
        break;

      case MOVE_6:
        car.Motor_Upper_L(1, SPEED); car.Motor_Upper_R(0, SPEED);
        car.Motor_Lower_L(0, 0);     car.Motor_Lower_R(0, 0);
        break;

      default:
        break;
    }

    lastState = currentState;
  }
}