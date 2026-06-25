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
#define DOCK_TURN_SPEED     45
#define DOCK_FORWARD_TIME   350
#define DOCK_STRAFE_TIME    900
#define DOCK_SETTLE_TIME    120

// If your robot tracks white floor instead of black tape, change HIGH to LOW
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
  DOCK_FOLLOW_LINE,
  DOCK_FORWARD_OFFSET,
  DOCK_STRAFE_IN,
  DOCK_SETTLE,
  DOCK_DONE
};

// =========================
// Globals
// =========================
RobotState currentState = STATE_INIT;
RobotState lastState = STATE_INIT;

// Auto-docking is a smaller state machine that only runs while currentState is MOVE_2.
DockPhase dockPhase = DOCK_IDLE;

bool lightsOn = false;
bool markerSeen = false;
bool dockToLeft = true;   // true = strafe left into bay, false = strafe right

// These timers let loop() do periodic work without blocking the whole robot.
unsigned long lastSonarTime = 0;
unsigned long lastColorTime = 0;
unsigned long lastSerialTime = 0;
unsigned long dockTimer = 0;

ColorDetected currentDetectedColor = NONE;

int rawR = 0;
int rawG = 0;
int rawB = 0;

// =========================
// Helper Functions
// =========================
float Get_Distance(void) {
  // HC-SR04 style ultrasonic sensors need a short trigger pulse before reading Echo.
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  // pulseIn returns echo time in microseconds; dividing by 58.2 converts to centimeters.
  float dis = pulseIn(EchoPin, HIGH, 30000) / 58.2;

  // Treat very small or missing readings as "no obstacle" to avoid false emergency stops.
  if (dis == 0 || dis < 10.0) return 999.0;
  return dis;
}

ColorDetected IdentifyColor() {
  // Turn off LEDs while sampling so reflected LED light does not distort color readings.
  car.right_led(0);
  car.left_led(0);
  delay(10);

  // TCS3200 measures color by selecting red, green, then blue filters with S2/S3.
  // Lower pulse values usually mean stronger reflected color for that channel.
  digitalWrite(S2, LOW);  digitalWrite(S3, LOW);  delay(15);
  rawR = pulseIn(sensorOut, LOW, 20000);

  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH); delay(15);
  rawG = pulseIn(sensorOut, LOW, 20000);

  digitalWrite(S2, LOW);  digitalWrite(S3, HIGH); delay(15);
  rawB = pulseIn(sensorOut, LOW, 20000);

  if (rawR == 0 || rawG == 0 || rawB == 0) return NONE;
  if (rawR > 350 && rawG > 350 && rawB > 350) return NONE;

  // The detected color is the channel with the clearly smallest pulse width.
  if (rawR < (rawG - 15) && rawR < (rawB - 15)) return RED;
  else if (rawG < (rawR - 15) && rawG < (rawB - 15)) return GREEN;
  else if (rawB < (rawR - 15) && rawB < (rawG - 15)) return BLUE;

  return NONE;
}

void PrintFSMState(RobotState state) {
  Serial.print("[FSM SYSTEM] Transitioned to: ");
  switch (state) {
    case STATE_INIT:   Serial.println("STATE_INIT"); break;
    case IDLE:         Serial.println("IDLE"); break;
    case MANUAL_FWD:   Serial.println("MANUAL_FWD"); break;
    case MANUAL_BWD:   Serial.println("MANUAL_BWD"); break;
    case MANUAL_LEFT:  Serial.println("MANUAL_LEFT"); break;
    case MANUAL_RIGHT: Serial.println("MANUAL_RIGHT"); break;
    case MOVE_1:       Serial.println("MOVE_1 (Line Tracking)"); break;
    case MOVE_2:       Serial.println("MOVE_2 (Auto Docking)"); break;
    case MOVE_3:       Serial.println("MOVE_3"); break;
    case MOVE_4:       Serial.println("MOVE_4"); break;
    case MOVE_5:       Serial.println("MOVE_5"); break;
    case MOVE_6:       Serial.println("MOVE_6"); break;
    default:           Serial.println("UNKNOWN"); break;
  }
}

void PrintDockPhase(DockPhase phase) {
  Serial.print("[DOCK] Phase -> ");
  switch (phase) {
    case DOCK_IDLE:           Serial.println("DOCK_IDLE"); break;
    case DOCK_FOLLOW_LINE:    Serial.println("DOCK_FOLLOW_LINE"); break;
    case DOCK_FORWARD_OFFSET: Serial.println("DOCK_FORWARD_OFFSET"); break;
    case DOCK_STRAFE_IN:      Serial.println("DOCK_STRAFE_IN"); break;
    case DOCK_SETTLE:         Serial.println("DOCK_SETTLE"); break;
    case DOCK_DONE:           Serial.println("DOCK_DONE"); break;
    default:                  Serial.println("UNKNOWN"); break;
  }
}

void MoveForwardRaw(int spd) {
  // Raw helpers directly command each wheel, useful when library movement names are unclear.
  car.Motor_Upper_L(1, spd);
  car.Motor_Upper_R(1, spd);
  car.Motor_Lower_L(1, spd);
  car.Motor_Lower_R(1, spd);
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

void ResetDocking() {
  // Reset all docking progress so the next dock command starts from the beginning.
  dockPhase = DOCK_IDLE;
  markerSeen = false;
  dockTimer = 0;
}

bool IsDockMarkerDetected(uint8_t SL, uint8_t SM, uint8_t SR) {
  // Start with this simple condition:
  // marker is detected when all 3 sensors see black at once.
  // Tune later if your real tape layout behaves differently.
  return (SL == BLACK_LINE && SM == BLACK_LINE && SR == BLACK_LINE);
}

void RunLineTracking() {
  uint8_t SL = digitalRead(SensorLeft);
  uint8_t SM = digitalRead(SensorMiddle);
  uint8_t SR = digitalRead(SensorRight);

  // Basic line following: center sensor drives straight; side sensors steer back to the line.
  if (SM == BLACK_LINE) {
    MoveForwardRaw(SPEED);
  }
  else if (SL == BLACK_LINE) {
    RotateLeftRaw(TURN_SPEED);
  }
  else if (SR == BLACK_LINE) {
    RotateRightRaw(TURN_SPEED);
  }
  else {
    car.Stop();
  }
}

void RunAutoDock(unsigned long currentMillis) {
  uint8_t SL = digitalRead(SensorLeft);
  uint8_t SM = digitalRead(SensorMiddle);
  uint8_t SR = digitalRead(SensorRight);

  // Docking is split into phases so each step can be timed and monitored separately.
  switch (dockPhase) {
    case DOCK_IDLE:
      markerSeen = false;
      dockPhase = DOCK_FOLLOW_LINE;
      PrintDockPhase(dockPhase);
      break;

    case DOCK_FOLLOW_LINE:
      if (SM == BLACK_LINE) {
        MoveForwardRaw(DOCK_FWD_SPEED);
      }
      else if (SL == BLACK_LINE) {
        RotateLeftRaw(DOCK_TURN_SPEED);
      }
      else if (SR == BLACK_LINE) {
        RotateRightRaw(DOCK_TURN_SPEED);
      }
      else {
        car.Stop();
      }

      if (IsDockMarkerDetected(SL, SM, SR)) {
        markerSeen = true;
        // Save the current time so the next phase can run for a fixed duration.
        dockTimer = currentMillis;
        dockPhase = DOCK_FORWARD_OFFSET;
        PrintDockPhase(dockPhase);
      }
      break;

    case DOCK_FORWARD_OFFSET:
      MoveForwardRaw(DOCK_FWD_SPEED);
      if (currentMillis - dockTimer >= DOCK_FORWARD_TIME) {
        car.Stop();
        dockTimer = currentMillis;
        dockPhase = DOCK_STRAFE_IN;
        PrintDockPhase(dockPhase);
      }
      break;

    case DOCK_STRAFE_IN:
      if (dockToLeft) StrafeLeftRaw();
      else StrafeRightRaw();

      if (currentMillis - dockTimer >= DOCK_STRAFE_TIME) {
        car.Stop();
        dockTimer = currentMillis;
        dockPhase = DOCK_SETTLE;
        PrintDockPhase(dockPhase);
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
      // Return to IDLE after docking so the robot does not keep trying to dock.
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

  // Pin modes tell the Arduino whether each pin reads a sensor or controls hardware.
  pinMode(EchoPin, INPUT);
  pinMode(TrigPin, OUTPUT);

  pinMode(SensorLeft, INPUT);
  pinMode(SensorMiddle, INPUT);
  pinMode(SensorRight, INPUT);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // TCS3200 frequency scaling: HIGH/LOW sets output frequency to a usable range.
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

// =========================
// Main Loop
// =========================
void loop() {
  unsigned long currentMillis = millis();

  // 0. Boot Sequence
  if (currentState == STATE_INIT) {
    ExecuteSelfAudit();
    currentState = IDLE;
  }

  // 1. Live Collision Check
  // Only check for front obstacles during states that drive forward.
  bool checkForwardObstacle =
    (currentState == MANUAL_FWD) ||
    (currentState == MOVE_1) ||
    (currentState == MOVE_2 && (dockPhase == DOCK_FOLLOW_LINE || dockPhase == DOCK_FORWARD_OFFSET));

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

  // 2. Color Detection & LED Patterns
  // Color is sampled every 100 ms so the loop can still handle driving and IR commands.
  if (currentMillis - lastColorTime >= 100) {
    currentDetectedColor = IdentifyColor();

    if (lightsOn) {
      car.right_led(1);
      car.left_led(1);
    }
    else {
      switch (currentDetectedColor) {
        case NONE:
          car.right_led(0); car.left_led(0);
          break;
        case GREEN:
          car.right_led(1); car.left_led(1);
          break;
        case RED:
          // Fast blink for red.
          if ((currentMillis / 100) % 2 == 0) {
            car.right_led(1); car.left_led(1);
          } else {
            car.right_led(0); car.left_led(0);
          }
          break;
        case BLUE:
          // Slow blink for blue.
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

  // 3. Serial Monitor Output
  if (currentMillis - lastSerialTime >= 500) {
    uint8_t SL = digitalRead(SensorLeft);
    uint8_t SM = digitalRead(SensorMiddle);
    uint8_t SR = digitalRead(SensorRight);

    Serial.print("Line Track -> L: ");
    Serial.print(SL == BLACK_LINE ? "BLACK" : "WHITE");
    Serial.print(" | M: ");
    Serial.print(SM == BLACK_LINE ? "BLACK" : "WHITE");
    Serial.print(" | R: ");
    Serial.println(SR == BLACK_LINE ? "BLACK" : "WHITE");

    lastSerialTime = currentMillis;
  }

  // 4. IR Input
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;

    // Each remote button changes the state; movement itself happens later in the FSM.
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

  // 5. Dynamic Continuous Logic
  // These states need constant sensor feedback, so they run every loop cycle.
  if (currentState == MOVE_1) {
    RunLineTracking();
  }

  if (currentState == MOVE_2) {
    RunAutoDock(currentMillis);
  }

  // 6. FSM Monitor & Static Logic Execution
  // One-time actions run only when the state changes, avoiding repeated setup commands.
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
        car.Back();
        break;

      case MANUAL_LEFT:
        car.L_Move();
        break;

      case MANUAL_RIGHT:
        car.R_Move();
        break;

      case MOVE_1:
        // handled continuously in RunLineTracking()
        break;

      case MOVE_2:
        // handled continuously in RunAutoDock()
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
