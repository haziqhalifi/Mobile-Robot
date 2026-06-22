#include <MecanumCar_v2.h>
mecanumCar mecanumCar(3, 2);  //sda-->D3,scl-->D2
#include <IRremote.h>

int RECV_PIN = A3;
IRrecv irrecv(RECV_PIN);
decode_results results;
bool flag = true;

const unsigned long TWO_STEP_TIME_MS = 2000;
uint8_t routeStage = 0;
unsigned long stageStartedAt = 0;

void startRoute() {
  routeStage = 1;
  stageStartedAt = millis();
  mecanumCar.Advance();
}

void stopRoute() {
  routeStage = 0;
  mecanumCar.Stop();
}

void updateRoute() {
  if (routeStage == 0 ||
      millis() - stageStartedAt < TWO_STEP_TIME_MS) {
    return;
  }

  stageStartedAt = millis();

  if (routeStage == 1) {
    routeStage = 2;
    mecanumCar.L_Move();
  } else if (routeStage == 2) {
    routeStage = 3;
    mecanumCar.Advance();
  } else {
    stopRoute();
  }
}

void setup(){
  Serial.begin(9600);
  mecanumCar.Init(); //Initialize the motor and the color light driver
  irrecv.enableIRIn();
  Serial.println("IR receiver ready");
}

void loop() {
  if (irrecv.decode(&results)) {
    Serial.print("IR code: 0x");
    Serial.println(results.value, HEX);

    if (results.value == 0xFF629D) // Up button: move forward
    {
    startRoute();
    }
    else if (results.value == 0xFFA857) // Stop button
    {
    stopRoute();
    }
    else if (results.value == 0xFF02FD && flag == true) //The value for turning on the light
    {
    mecanumCar.right_led(1);
    mecanumCar.left_led(1);
    flag = false;

    }
     else if (results.value == 0xFF02FD && flag == false) //The value for turning off the lights
    {
    mecanumCar.right_led(0);
    mecanumCar.left_led(0);
    flag = true;
    }
    irrecv.resume(); // Receive the next value
  }

  updateRoute();
}
