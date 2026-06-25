#include <Arduino.h>

constexpr uint8_t EchoPin = 13;
constexpr uint8_t TrigPin = 12;

float Get_Distance();

void setup()
{
  Serial.begin(9600);
  pinMode(EchoPin, INPUT);
  pinMode(TrigPin, OUTPUT);
}

void loop()
{
  const float distance = Get_Distance();

  Serial.print("distance: ");
  if (distance < 0.0F) {
    Serial.println("out of range");
  } else {
    Serial.print(distance);
    Serial.println(" cm");
  }

  delay(100);
}

float Get_Distance()
{
  digitalWrite(TrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPin, LOW);

  const unsigned long duration = pulseIn(EchoPin, HIGH, 30000UL);
  if (duration == 0) {
    return -1.0F;
  }

  return duration / 58.2F;
}
