#include "AS5600.h"

AS5600 ASL1;
AS5600 ASL2;

double position = 0;

void setup() {
  Serial.begin(9600);
  delay(2000);
  Wire.begin();

  Serial.print("ADDR: ");
  Serial.println(ASL1.getAddress());
  Serial.println(ASL2.getAddress());

  pinMode(6, OUTPUT); // angle
  pinMode(5, OUTPUT); // angle
  pinMode(9, OUTPUT); // speed

 pinMode(LED_BUILTIN, OUTPUT);
}

int target_position = 100;
int target_speed = 0;

void loop() {
  // read the incoming byte:
  if (Serial.available() > 0) {
    unsigned char buf[2];
    size_t size = Serial.readBytes(buf, 2);

    target_position = map((unsigned int) buf[1], 0, 255, 0, 360);
    target_speed =   min((unsigned int) buf[0], 255);
    Serial.println((unsigned int)buf[1]);
    Serial.println((unsigned int)buf[0]);

    digitalWrite(LED_BUILTIN, HIGH);
  }else{
    digitalWrite(LED_BUILTIN, LOW);
  }

  moveToTarget(target_position);
  analogWrite(9, map(target_speed, 0, 255, 0, 100));

  position = modulo(int(ASL1.getCumulativePosition()*2 * AS5600_RAW_TO_DEGREES), 360);
}

void moveToTarget(long target) {
  long angleDifference = target - position;
  int direction = 1;  // 1 for clockwise, -1 for counterclockwise

  // Calculate shortest path taking wrap-around into account
  if (angleDifference < -180) {
    angleDifference += 360;
  } else if (angleDifference > 180) {
    angleDifference -= 360;
  }

  direction = angleDifference > 0 ? 1 : -1;

  int pwmValue = map(abs(angleDifference), 0, 360, 0, 10);

  // Set motor direction based on shortest path
  if (direction == 1) {
    digitalWrite(6, pwmValue);
    digitalWrite(5, LOW);
  } else {
    digitalWrite(6, LOW);
    digitalWrite(5, pwmValue);
  }

}

int modulo(int n, int M){
  return ((n % M) + M) % M;
}
