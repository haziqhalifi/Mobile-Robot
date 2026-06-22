#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MecanumCar_v2.h>
#include <IRremote.h>

// IR Hex Codes
#define BTN_1     0xFF6897
#define BTN_2     0xFF9867
#define BTN_3     0xFFB04F
#define BTN_4     0xFF30CF
#define BTN_5     0xFF18E7
#define BTN_6     0xFF7A85
#define BTN_7     0xFF10EF

#define BTN_UP    0xFF629D
#define BTN_DOWN  0xFFA857
#define BTN_LEFT  0xFF22DD
#define BTN_RIGHT 0xFFC23D
#define BTN_0     0xFF4AB5
#define BTN_HASH  0xFF52AD
#define BTN_OK    0xFF02FD

#define IR_REPEAT 0xFFFFFFFF
#define IR_PIN A3

#define NORMAL_SPEED 100
#define ARC_FAST_SPEED 120
#define ARC_SLOW_SPEED 45

mecanumCar car(3, 2);
Adafruit_PWMServoDriver armDriver(0x40);
IRrecv irrecv(IR_PIN);
decode_results results;

// Four-servo arm on PCA9685 channels 0-3.
constexpr uint8_t ARM_BASE_CHANNEL = 0;
constexpr uint8_t ARM_SHOULDER_CHANNEL = 1;
constexpr uint8_t ARM_ELBOW_CHANNEL = 2;
constexpr uint8_t ARM_GRIPPER_CHANNEL = 3;
constexpr uint16_t SERVO_MIN_PULSE = 110;
constexpr uint16_t SERVO_MAX_PULSE = 500;

// Calibrate these angles for the arm before lifting a real object.
constexpr uint8_t BASE_PICK = 55;
constexpr uint8_t BASE_PLACE = 125;
constexpr uint8_t SHOULDER_UP = 70;
constexpr uint8_t SHOULDER_DOWN = 125;
constexpr uint8_t ELBOW_UP = 105;
constexpr uint8_t ELBOW_DOWN = 55;
constexpr uint8_t GRIPPER_OPEN = 80;
constexpr uint8_t GRIPPER_CLOSED = 35;

// FSM States
enum RobotState {
  IDLE,
  WAYPOINT_1,
  MANUAL_FWD,
  MANUAL_BWD,
  MANUAL_LEFT,
  MANUAL_RIGHT,
  DIAGONAL_FWD_LEFT,
  DIAGONAL_FWD_RIGHT,
  ARC_LEFT,
  ARC_RIGHT,
  ROTATE_CW,
  ROTATE_CCW,
  PICK_AND_PLACE
};

RobotState currentState = IDLE;

bool lightsOn = false;
unsigned long lastCode = 0;
uint8_t armStage = 0;
unsigned long armStageStartedAt = 0;

void setArmServo(uint8_t channel, uint8_t angle) {
  angle = constrain(angle, 0, 180);
  const uint16_t pulse = map(angle, 0, 180, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
  armDriver.setPWM(channel, 0, pulse);
}

void setArmPose(uint8_t base, uint8_t shoulder, uint8_t elbow,
                uint8_t gripper) {
  setArmServo(ARM_BASE_CHANNEL, base);
  setArmServo(ARM_SHOULDER_CHANNEL, shoulder);
  setArmServo(ARM_ELBOW_CHANNEL, elbow);
  setArmServo(ARM_GRIPPER_CHANNEL, gripper);
}

void startPickAndPlace() {
  car.Stop();
  armStage = 1;
  armStageStartedAt = millis();
  // Start above the pickup point with the gripper open.
  setArmPose(BASE_PICK, SHOULDER_UP, ELBOW_UP, GRIPPER_OPEN);
}

void stopPickAndPlace() {
  armStage = 0;
  currentState = IDLE;
}

void updatePickAndPlace() {
  if (armStage == 0 || millis() - armStageStartedAt < 900UL) {
    return;
  }

  armStageStartedAt = millis();

  switch (armStage++) {
    case 1: // Lower toward the object.
      setArmPose(BASE_PICK, SHOULDER_DOWN, ELBOW_DOWN, GRIPPER_OPEN);
      break;
    case 2: // Pick it up.
      setArmPose(BASE_PICK, SHOULDER_DOWN, ELBOW_DOWN, GRIPPER_CLOSED);
      break;
    case 3: // Lift it.
      setArmPose(BASE_PICK, SHOULDER_UP, ELBOW_UP, GRIPPER_CLOSED);
      break;
    case 4: // Rotate toward the placement point.
      setArmPose(BASE_PLACE, SHOULDER_UP, ELBOW_UP, GRIPPER_CLOSED);
      break;
    case 5: // Lower it.
      setArmPose(BASE_PLACE, SHOULDER_DOWN, ELBOW_DOWN, GRIPPER_CLOSED);
      break;
    case 6: // Release it.
      setArmPose(BASE_PLACE, SHOULDER_DOWN, ELBOW_DOWN, GRIPPER_OPEN);
      break;
    case 7: // Return to a raised resting pose.
      setArmPose(BASE_PICK, SHOULDER_UP, ELBOW_UP, GRIPPER_OPEN);
      break;
    default:
      stopPickAndPlace();
      break;
  }
}

// Direct wheel control helper
void setWheels(int upperLeftDir, int upperLeftSpeed,
               int upperRightDir, int upperRightSpeed,
               int lowerLeftDir, int lowerLeftSpeed,
               int lowerRightDir, int lowerRightSpeed) {
  car.Motor_Upper_L(upperLeftDir, upperLeftSpeed);
  car.Motor_Upper_R(upperRightDir, upperRightSpeed);
  car.Motor_Lower_L( lowerLeftDir, lowerLeftSpeed);
  car.Motor_Lower_R( lowerRightDir, lowerRightSpeed);
}

// Picture b style: strafe right
void moveRight() {
  car.R_Move();
}

// Picture b opposite: strafe left
void moveLeft() {
  car.L_Move();
}

// Picture c style: diagonal forward-right
void diagonalForwardRight() {
  setWheels(
    1, NORMAL_SPEED,
    1, 0,
    1, 0,
    1, NORMAL_SPEED
  );
}

// Diagonal forward-left
void diagonalForwardLeft() {
  setWheels(
    1, 0,
    1, NORMAL_SPEED,
    1, NORMAL_SPEED,
    1, 0
  );
}

// Picture e style: rotate clockwise
void rotateClockwise() {
  setWheels(
    1, NORMAL_SPEED,
    0, NORMAL_SPEED,
    1, NORMAL_SPEED,
    0, NORMAL_SPEED
  );
}

// Rotate counterclockwise
void rotateCounterClockwise() {
  setWheels(
    0, NORMAL_SPEED,
    1, NORMAL_SPEED,
    0, NORMAL_SPEED,
    1, NORMAL_SPEED
  );
}

// Picture d style: curved/arc left
void arcLeft() {
  setWheels(
    1, ARC_SLOW_SPEED,
    1, ARC_FAST_SPEED,
    1, ARC_SLOW_SPEED,
    1, ARC_FAST_SPEED
  );
}

// Picture f style: curved/arc right
void arcRight() {
  setWheels(
    1, ARC_FAST_SPEED,
    1, ARC_SLOW_SPEED,
    1, ARC_FAST_SPEED,
    1, ARC_SLOW_SPEED
  );
}

// Custom stopwatch function for sequences
bool safeDelay(unsigned long waitTime) {
  unsigned long start = millis();

  while (millis() - start < waitTime) {
    if (irrecv.decode(&results)) {
      unsigned long code = results.value;

      if (code == IR_REPEAT) {
        code = lastCode;
      } else {
        lastCode = code;
      }

      if (code == BTN_0 || code == BTN_HASH) {
        car.Stop();
        currentState = IDLE;
        irrecv.resume();
        return true;
      }

      irrecv.resume();
    }
  }

  return false;
}

void setup() {
  Serial.begin(9600);
  car.Init();
  armDriver.begin();
  armDriver.setPWMFreq(50);
  delay(10);
  setArmPose(BASE_PICK, SHOULDER_UP, ELBOW_UP, GRIPPER_OPEN);
  irrecv.enableIRIn();
}

void loop() {
  // 1. Digital Perception
  if (irrecv.decode(&results)) {
    unsigned long code = results.value;

    if (code == IR_REPEAT) {
      code = lastCode;
    } else {
      lastCode = code;
    }

    // Toggle headlights
    if (code == BTN_OK) {
      lightsOn = !lightsOn;
      car.right_led(lightsOn);
      car.left_led(lightsOn);
      delay(300);
    }

    // Universal stop
    else if (code == BTN_0 || code == BTN_HASH) {
      car.Stop();
      currentState = IDLE;
      delay(300);
    }

    // Waypoint sequence
    else if (code == BTN_1) {
      currentState = WAYPOINT_1;
      delay(300);
    }

    // Basic manual movement
    else if (code == BTN_UP) {
      currentState = MANUAL_FWD;
      delay(300);
    }
    else if (code == BTN_DOWN) {
      currentState = MANUAL_BWD;
      delay(300);
    }
    else if (code == BTN_LEFT) {
      currentState = MANUAL_LEFT;
      delay(300);
    }
    else if (code == BTN_RIGHT) {
      currentState = MANUAL_RIGHT;
      delay(300);
    }

    // Extra mecanum movement from picture
    else if (code == BTN_2) {
      currentState = DIAGONAL_FWD_LEFT;
      delay(300);
    }
    else if (code == BTN_3) {
      currentState = DIAGONAL_FWD_RIGHT;
      delay(300);
    }
    else if (code == BTN_4) {
      currentState = PICK_AND_PLACE;
      startPickAndPlace();
      delay(300);
    }
    else if (code == BTN_5) {
      currentState = ROTATE_CW;
      delay(300);
    }
    else if (code == BTN_6) {
      currentState = ARC_RIGHT;
      delay(300);
    }
    else if (code == BTN_7) {
      currentState = ROTATE_CCW;
      delay(300);
    }

    irrecv.resume();
  }

  // 2. Logic Execution
  switch (currentState) {
    case IDLE:
      car.Stop();
      break;

    case MANUAL_FWD:
      car.Advance();
      break;

    case MANUAL_BWD:
      car.Back();
      break;

    case MANUAL_LEFT:
      moveLeft();
      break;

    case MANUAL_RIGHT:
      moveRight();
      break;

    case DIAGONAL_FWD_LEFT:
      diagonalForwardLeft();
      break;

    case DIAGONAL_FWD_RIGHT:
      diagonalForwardRight();
      break;

    case ARC_LEFT:
      arcLeft();
      break;

    case ARC_RIGHT:
      arcRight();
      break;

    case ROTATE_CW:
      rotateClockwise();
      break;

    case ROTATE_CCW:
      rotateCounterClockwise();
      break;

    case PICK_AND_PLACE:
      car.Stop();
      updatePickAndPlace();
      break;

    case WAYPOINT_1:
      car.Advance();
      if (safeDelay(2000)) {
        currentState = IDLE;
        break;
      }

      moveLeft();
      if (safeDelay(2000)) {
        currentState = IDLE;
        break;
      }

      car.Advance();
      if (safeDelay(2000)) {
        currentState = IDLE;
        break;
      }

      rotateClockwise();
      if (safeDelay(1200)) {
        currentState = IDLE;
        break;
      }

      diagonalForwardRight();
      if (safeDelay(1500)) {
        currentState = IDLE;
        break;
      }

      car.Stop();
      currentState = IDLE;
      break;
  }
}
