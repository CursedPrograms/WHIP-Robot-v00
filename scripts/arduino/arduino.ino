/*
  Hexapod - Rest -> Stretch -> Stand sequence, then live sensor readout
  Controller: RTrobot Servo Motor Controller (32 channel)
  Wiring: SoftwareSerial - Arduino pin 11 (RX) = controller TX,
                            Arduino pin 10 (TX) = controller RX

  Sends three poses in order, waiting for each move to finish before
  sending the next:
    1. Rest    - legs only (channels 1-9 + 24-32)
    2. Stretch - legs fully extended (channels 1-9 + 24-32)
    3. Stand   - all 32 channels

  After the sequence finishes, loop() continuously prints ultrasonic
  distance + MPU6050 accel/gyro readings to the Serial Monitor.

  Servos:
    1. L Back Coxa      2. L Back Femur      3. L Back Tibia
    4. L Middle Coxa    5. L Middle Femur    6. L Middle Tibia
    7. L Front Coxa     8. L Front Femur     9. L Front Tibia
   24. R Front Coxa    25. R Front Femur    26. R Front Tibia
   27. R Middle Coxa   28. R Middle Femur   29. R Middle Tibia
   30. R Back Coxa     31. R Back Femur     32. R Back Tibia

  Sensor wiring:
    HC-SR04: TRIG -> Arduino pin 7, ECHO -> Arduino pin 6
    MPU6050: SDA -> Arduino A4, SCL -> Arduino A5, VCC -> 5V, GND -> GND
*/

#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial controllerSerial(11, 10); // RX, TX

const int TRIG_PIN = 7;
const int ECHO_PIN = 6;
const int MPU_ADDR = 0x68; // MPU6050 I2C address (AD0 pin low/unconnected)

int16_t rawAccX, rawAccY, rawAccZ;
int16_t rawGyroX, rawGyroY, rawGyroZ;

const char STAND_UP_CMD[] =
  "#1P2500#2P1500#3P1500#4P1500#5P1500#6P1500#7P800#8P1500#9P1500"
  "#10P1500#11P1500#12P1500#13P1500#14P1500#15P1500#16P1500#17P1500#18P1500"
  "#19P1500#20P1500#21P1500#22P1500#23P1500"
  "#24P2200#25P1500#26P1500#27P1500#28P1500#29P1500#30P500#31P1500#32P1500"
  "T500D500\r\n";

const char REST_CMD[] =
  "#2P2500#3P2500#5P2500#6P2500#7P800#8P2500#9P2500"
  "#25P500#26P500#28P500#29P500#31P500#32P500"
  "T500D500\r\n";

const char STRETCH_CMD[] =
  "#1P708#2P1643#3P630#4P526#5P1643#6P630#7P500#8P1643#9P630"
  "#24P2292#25P1201#26P2344#27P2240#28P1201#29P2344#30P2500#31P1201#32P2344"
  "T500D500\r\n";

const int MOVE_SETTLE_MS = 500 + 500; // T500 + D500

void sendPose(const char* cmd, const char* label) {
  controllerSerial.print(cmd);
  Serial.print("Sent: ");
  Serial.println(label);
  delay(MOVE_SETTLE_MS + 200);
}

void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // wake up the MPU6050 (clears the sleep bit)
  Wire.endTransmission(true);
}

void mpuRead() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);

  rawAccX = Wire.read() << 8 | Wire.read();
  rawAccY = Wire.read() << 8 | Wire.read();
  rawAccZ = Wire.read() << 8 | Wire.read();
  Wire.read(); Wire.read(); // discard temperature
  rawGyroX = Wire.read() << 8 | Wire.read();
  rawGyroY = Wire.read() << 8 | Wire.read();
  rawGyroZ = Wire.read() << 8 | Wire.read();
}

long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;

  return duration * 0.034 / 2;
}

void printSensorReadings() {
  long distanceCM = readDistanceCM();
  mpuRead();

  float accXg = rawAccX / 16384.0;
  float accYg = rawAccY / 16384.0;
  float accZg = rawAccZ / 16384.0;
  float gyroXdps = rawGyroX / 131.0;
  float gyroYdps = rawGyroY / 131.0;
  float gyroZdps = rawGyroZ / 131.0;

  Serial.print("Distance: ");
  if (distanceCM < 0) {
    Serial.print("out of range");
  } else {
    Serial.print(distanceCM);
    Serial.print(" cm");
  }

  Serial.print("  |  Accel (g) X:");
  Serial.print(accXg, 2);
  Serial.print(" Y:");
  Serial.print(accYg, 2);
  Serial.print(" Z:");
  Serial.print(accZg, 2);

  Serial.print("  |  Gyro (deg/s) X:");
  Serial.print(gyroXdps, 1);
  Serial.print(" Y:");
  Serial.print(gyroYdps, 1);
  Serial.print(" Z:");
  Serial.println(gyroZdps, 1);
}

void setup() {
  Serial.begin(9600);            // USB debug monitor
  controllerSerial.begin(9600);  // must match controller's UART baud rate

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  Wire.begin();
  mpuInit();

  delay(2000); // let the controller finish booting

  sendPose(REST_CMD, "rest pose");
  sendPose(STRETCH_CMD, "stretch pose");
  sendPose(STAND_UP_CMD, "stand up pose");
}

void loop() {
  // Echo any reply from the controller (e.g. "OK") to the Serial Monitor
  while (controllerSerial.available()) {
    char c = controllerSerial.read();
    Serial.write(c);
  }

  // Print sensor readings every 300ms, without blocking controller replies
  static unsigned long lastSensorRead = 0;
  if (millis() - lastSensorRead >= 300) {
    lastSensorRead = millis();
    printSensorReadings();
  }
}
