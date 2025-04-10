#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Addresses
#define MUX_ADDR     0x70
#define AS5600_ADDR  0x36

// Registers for AS5600 angle reading (high/low bytes)
#define AS5600_ANGLE_HIGH 0x0E
#define AS5600_ANGLE_LOW  0x0F


Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40); // Default I2C address
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x41); // Default I2C address

// Keep track of last raw angle and cumulative angle (in degrees) for each sensor
uint16_t lastRawAngle[8];
float cumulativeAngle[8];
double GEAR_RATIO = 34.0/18.0;

// Select which channel on the PCA9548A to use
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel); 
  Wire.endTransmission();
  delay(1);
}

// Read 12-bit angle from AS5600
uint16_t readAS5600AngleRaw() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_ANGLE_HIGH);
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_ADDR, 2);

  uint8_t highByte = Wire.read();
  uint8_t lowByte  = Wire.read();
  return ((highByte & 0x0F) << 8) | lowByte; // 12-bit angle
}

void setPWM(int channel, int duty){
  if(channel <= 15){
    pwm1.setPWM(channel, 0, duty);
  }else{
    pwm2.setPWM(channel-16, 0, duty);
  }
  
  delay(1);
}

void setup() {
  // delay(2000);  // Wait 2 seconds before doing anything
  Wire.begin();
  Serial.begin(9600);
  // while (!Serial); // Wait for serial monitor
  Serial.println("Starting...");

  pwm1.begin();
  pwm1.setPWMFreq(200); // set pwm frequency
  pwm2.begin();
  pwm2.setPWMFreq(200); // set pwm frequency

  // Initialize last raw angle with current readings
  for(int i = 0; i < 8; i++){
    selectMuxChannel(i);
    lastRawAngle[i] = readAS5600AngleRaw();
  }

  pinMode(LED_BUILTIN, OUTPUT);
  for(int i = 0; i < 32; i++){
    setPWM(i, 0);
  }
}

int target = 0;
long startTime = millis();

int target_position = 0;
int target_speed = 0;
void loop() {
  if (Serial.available() > 0) {
    unsigned char buf[3];
    size_t size = Serial.readBytes(buf, 3);

    int code = buf[0];

    if(code == 1){
      target_position = map((unsigned int) buf[2], 0, 255, 0, 360);
      target_speed =   min((unsigned int) buf[1], 255);
      Serial.println((unsigned int)buf[2]);
      Serial.println((unsigned int)buf[1]);
    }else if(code == 2){
      // nudging code
      int nudge_idx = buf[2];
      int nudge_dir = buf[1];
      if(nudge_dir == 1){
        cumulativeAngle[nudge_idx] += 10;
      }else if(nudge_dir == 2){
        cumulativeAngle[nudge_idx] -= 10;
      }
    }else if(code == 3){
        for(int i = 0; i < 8; i++){
          cumulativeAngle[i] = 0;
        }
    }



    digitalWrite(LED_BUILTIN, HIGH);
  }else{
    digitalWrite(LED_BUILTIN, LOW);
  }
  // if(millis()-startTime > 400){
  //   target = (target+50) % 360;
  //   startTime = millis();
  // }
  // Serial.println(target);
  updatePositions();
  moveToTarget(1, 0, 7, target_position);
  moveToTarget(5, 4, 4, target_position);
  moveToTarget(9, 8, 0, target_position);
  moveToTarget(13, 12, 2, target_position);
  moveToTarget(16, 17, 6, target_position);
  moveToTarget(20, 21, 5, target_position);
  // moveToTarget(13, 12, 3, target_position);
  // moveToTarget(13, 12, 3, target_position);
  int speed = max(min(4095, map(target_speed,0,255,0,4095)), 0);
  setPWM(2, speed);
  setPWM(6, speed);
  setPWM(10, speed);
  setPWM(14, speed);
  setPWM(18, speed);
  setPWM(22, speed);

  // for(int i = 0; i < 31; i+=2){
  //   setPWM(i, 2000);
  // }
  // setPWM(20,4095);
  printPositions();
}

void updatePositions(){
  uint16_t rawAngle[8];
  
  for(int i = 0; i < 8; i++){
    selectMuxChannel(i);
    rawAngle[i] = readAS5600AngleRaw();

    // Calculate difference
    int16_t diff = rawAngle[i] - lastRawAngle[i];
    // Handle wrap-around
    if (diff < -2048) diff += 4096;
    else if (diff > 2048) diff -= 4096;

    // Update cumulative angle
    cumulativeAngle[i] += diff * (360.0 / 4096.0);
    lastRawAngle[i] = rawAngle[i];
  }
}

void printPositions(){
  // Print cumulative angles
  for(int i = 0; i < 8; i++){
    Serial.print("Angle");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(cumulativeAngle[i]);
    Serial.print("° ");
  }
  Serial.println("");
}

int p = 50;
int d = 0.1;
int lastError[12];
void moveToTarget(int motorChannel1, int motorChannel2, int sensorChannel, long target) {
  long angleDifference = target - modulo(cumulativeAngle[sensorChannel] * GEAR_RATIO, 360);
  int direction = 1;  // 1 for clockwise, -1 for counterclockwise

  // Calculate shortest path taking wrap-around into account
  if (angleDifference < -180) {
    angleDifference += 360;
  } else if (angleDifference > 180) {
    angleDifference -= 360;
  }

  direction = angleDifference > 0 ? 1 : -1;

  // int pwmValue = max(0,min(4094, map(abs(angleDifference), 0, 50, 0, 4095)));t
  int pwmValue = max(0, min(4095, abs(angleDifference) * p));

  // Set motor direction based on shortest path
  if (direction == 1) {
    setPWM(motorChannel1, pwmValue);
    setPWM(motorChannel2, 0);
  } else {
    setPWM(motorChannel1, 0);
    setPWM(motorChannel2, pwmValue);
  }
}

int modulo(int n, int M){
  return ((n % M) + M) % M;
}
