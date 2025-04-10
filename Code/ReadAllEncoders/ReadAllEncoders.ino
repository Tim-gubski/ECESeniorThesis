#include <Wire.h>

// Addresses
#define MUX_ADDR     0x70
#define AS5600_ADDR  0x36

// Registers for AS5600 angle reading (high/low bytes)
#define AS5600_ANGLE_HIGH 0x0E
#define AS5600_ANGLE_LOW  0x0F

// Keep track of last raw angle and cumulative angle (in degrees) for each sensor
uint16_t lastRawAngle1, lastRawAngle2;
float cumulativeAngle1 = 0.0;
float cumulativeAngle2 = 0.0;

// Select which channel on the PCA9548A to use
void selectMuxChannel(uint8_t channel) {
  Wire.beginTransmission(MUX_ADDR);
  Wire.write(1 << channel); 
  Wire.endTransmission();
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

void setup() {
  Wire.begin();
  Serial.begin(115200);

  // Initialize last raw angle with current readings
  selectMuxChannel(0);
  lastRawAngle1 = readAS5600AngleRaw();

  selectMuxChannel(1);
  lastRawAngle2 = readAS5600AngleRaw();
}

void loop() {
  // For first sensor
  selectMuxChannel(0);
  uint16_t rawAngle1 = readAS5600AngleRaw();

  // Calculate difference
  int16_t diff1 = rawAngle1 - lastRawAngle1;
  // Handle wrap-around
  if (diff1 < -2048) diff1 += 4096;
  else if (diff1 > 2048) diff1 -= 4096;

  // Update cumulative angle
  cumulativeAngle1 += diff1 * (360.0 / 4096.0);
  lastRawAngle1 = rawAngle1;

  // For second sensor
  selectMuxChannel(1);
  uint16_t rawAngle2 = readAS5600AngleRaw();

  // Calculate difference
  int16_t diff2 = rawAngle2 - lastRawAngle2;
  // Handle wrap-around
  if (diff2 < -2048) diff2 += 4096;
  else if (diff2 > 2048) diff2 -= 4096;

  // Update cumulative angle
  cumulativeAngle2 += diff2 * (360.0 / 4096.0);
  lastRawAngle2 = rawAngle2;

  // Print cumulative angles
  Serial.print("Cumulative Angle1: ");
  Serial.print(cumulativeAngle1);
  Serial.print("°,  Angle2: ");
  Serial.print(cumulativeAngle2);
  Serial.println("°");

  delay(200);
}
