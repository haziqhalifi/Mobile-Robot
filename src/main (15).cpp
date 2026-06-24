#include <Arduino.h>
#include <MecanumCar_v2.h>
#include <IRremote.h>
#include <Servo.h> 

// IR Hex Codes
#define BTN_1     0xFF6897 // Starts Line Tracking & Intersection Counting
#define BTN_2     0xFF9867
#define BTN_3     0xFFB04F
#define BTN_4     0xFF30CF
#define BTN_5     0xFF18E7
#define BTN_6     0xFF7A85
#define BTN_UP    0xFF629D
#define BTN_DOWN  0xFFA857
#define BTN_LEFT  0xFF22DD
#define BTN_RIGHT 0xFFC23D
#define BTN_0     0xFF4AB5
#define BTN_OK    0xFF02FD 

// Hardware Pins
#define IR_PIN A3
#define EchoPin 13  
#define TrigPin 12  
#define SensorLeft    A0   
#define SensorMiddle  A1   
#define SensorRight   A2   
#define GripperPin    9    

// TCS3200 Color Sensor Pins 
#define S0 4
#define S1 5
#define S2 6
#define S3 7
#define sensorOut 8

// Configuration (Reduced by ~30%)
#define SPEED 56 
#define TURN_SPEED 35 
#define MICRO_TURN_SPEED 25 // Slower wheel speed for forward-curving corrections
#define STOP_DISTANCE 32.0 

// --- TCRT5000 LINE TRACKING LOGIC ---
#define BLACK_LINE HIGH 

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

RobotState currentState = STATE_INIT;
RobotState lastState = STATE_INIT; 

bool lightsOn = false; 
unsigned long lastSonarTime = 0; 
unsigned long lastColorTime = 0;
unsigned long lastSerialTime = 0; 

// Intersection Tracking Variables
int intersectionCount = 0;
bool isOnIntersection = false;

ColorDetected currentDetectedColor = NONE; 

int rawR = 0;
int rawG = 0;
int rawB = 0;

// --- Helper Functions ---

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
  Serial.print("[FSM SYSTEM] Transitioned to: ");
  switch(state) {
    case STATE_INIT:   Serial.println("STATE_INIT"); break;
    case IDLE:         Serial.println("IDLE"); break;
    case MANUAL_FWD:   Serial.println("MANUAL_FWD"); break;
    case MANUAL_BWD:   Serial.println("MANUAL_BWD"); break;
    case MANUAL_LEFT:  Serial.println("MANUAL_LEFT"); break;
    case MANUAL_RIGHT: Serial.println("MANUAL_RIGHT"); break;
    case MOVE_1:       Serial.println("MOVE_1 (Straight Line Track)"); break;
    case MOVE_2:       Serial.println("MOVE_2"); break;
    case MOVE_3:       Serial.println("MOVE_3"); break;
    case MOVE_4:       Serial.println("MOVE_4"); break;
    case MOVE_5:       Serial.println("MOVE_5"); break;
    case MOVE_6:       Serial.println("MOVE_6"); break;
    default:           Serial.println("UNKNOWN"); break;
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

  // Gripper Check moved to the very end of initialization
  Serial.print("[AUDIT] Gripper Servo... ");
  gripper.write(60); delay(300);
  gripper.write(90); delay(300);
  Serial.println("PASS (Sweep complete)");

  Serial.println("=============================================");
  Serial.println(">>> AUDIT COMPLETE. READY FOR COMMANDS.");
  Serial.println("=============================================\n");
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
        if (currentState != IDLE) Serial.println("[ALERT] Wall Detected! Emergency Stop.");
        currentState = IDLE; 
      }
      lastSonarTime = currentMillis;
    }
  }

  // 2. Color Detection & LED Patterns 
  // Adjusted to 250ms to unblock the processor and increase IR scan frequency by 100%
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
          else { car.right_led(0); car.left_led(0); } break;
        case BLUE:
          if ((currentMillis / 500) % 2 == 0) { car.right_led(1); car.left_led(1); } 
          else { car.right_led(0); car.left_led(0); } break;
      }
    }
    lastColorTime = currentMillis;
  }

  // 3. Serial Monitor Output (Line Tracking IR Sensors)
  // Adjusted to match new fast loop timings
  if (currentMillis - lastSerialTime >= 250) { 
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

  // 4. Digital Perception Block (IR)
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;
    
    if (code == BTN_OK) { lightsOn = !lightsOn; delay(300); }
    else if (code == BTN_0) { currentState = IDLE; delay(300); }
    else if (code == BTN_UP) { currentState = MANUAL_FWD; delay(300); }
    else if (code == BTN_DOWN) { currentState = MANUAL_BWD; delay(300); }
    else if (code == BTN_LEFT) { currentState = MANUAL_LEFT; delay(300); }
    else if (code == BTN_RIGHT) { currentState = MANUAL_RIGHT; delay(300); }
    else if (code == BTN_1) { 
      intersectionCount = 0; // Reset intersection counter for a fresh run
      isOnIntersection = false;
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

  // 5. Dynamic Continuous Logic (Line Tracking specific to TCRT5000)
  if (currentState == MOVE_1) {
    uint8_t SL = digitalRead(SensorLeft);
    uint8_t SM = digitalRead(SensorMiddle);
    uint8_t SR = digitalRead(SensorRight);

    // 1. INTERSECTION LOGIC: All three hit black -> Punch straight through & count interval
    if (SL == BLACK_LINE && SM == BLACK_LINE && SR == BLACK_LINE) {
      car.Motor_Upper_L(1, SPEED); car.Motor_Upper_R(1, SPEED);
      car.Motor_Lower_L(1, SPEED); car.Motor_Lower_R(1, SPEED);
      
      // Ensure we only count the horizontal line once per crossing
      if (!isOnIntersection) {
        intersectionCount++;
        Serial.print(">>> INTERSECTION PASSED! Total Count: ");
        Serial.println(intersectionCount);
        isOnIntersection = true;
      }
    } 
    else {
      // Robot has fully cleared the intersection, reset the flag
      isOnIntersection = false;

      // 2. PERFECT CENTER: White, Black, White -> Drive straight
      if (SL != BLACK_LINE && SM == BLACK_LINE && SR != BLACK_LINE) {
        car.Motor_Upper_L(1, SPEED); car.Motor_Upper_R(1, SPEED);
        car.Motor_Lower_L(1, SPEED); car.Motor_Lower_R(1, SPEED);
      } 
      // 3. MINOR DRIFT RIGHT: Black, Black, White -> Micro-turn Left (Forward Curve)
      else if (SL == BLACK_LINE && SM == BLACK_LINE && SR != BLACK_LINE) {
        car.Motor_Upper_L(1, MICRO_TURN_SPEED); car.Motor_Upper_R(1, SPEED);
        car.Motor_Lower_L(1, MICRO_TURN_SPEED); car.Motor_Lower_R(1, SPEED);
      }
      // 4. MINOR DRIFT LEFT: White, Black, Black -> Micro-turn Right (Forward Curve)
      else if (SL != BLACK_LINE && SM == BLACK_LINE && SR == BLACK_LINE) {
        car.Motor_Upper_L(1, SPEED); car.Motor_Upper_R(1, MICRO_TURN_SPEED);
        car.Motor_Lower_L(1, SPEED); car.Motor_Lower_R(1, MICRO_TURN_SPEED);
      }
      // 5. HARD DRIFT RIGHT: Black, White, White -> Hard Pivot Left to acquire line
      else if (SL == BLACK_LINE && SM != BLACK_LINE && SR != BLACK_LINE) {
        car.Motor_Upper_L(1, 0);          car.Motor_Upper_R(1, TURN_SPEED);
        car.Motor_Lower_L(1, 0);          car.Motor_Lower_R(1, TURN_SPEED);
      } 
      // 6. HARD DRIFT LEFT: White, White, Black -> Hard Pivot Right to acquire line
      else if (SL != BLACK_LINE && SM != BLACK_LINE && SR == BLACK_LINE) {
        car.Motor_Upper_L(1, TURN_SPEED); car.Motor_Upper_R(1, 0);
        car.Motor_Lower_L(1, TURN_SPEED); car.Motor_Lower_R(1, 0);
      } 
      // 7. LOST LOGIC: Entirely off the line -> Halt
      else { 
        car.Stop(); 
      }
    }
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