/*
  Hexapod - All Channels to Center (1500)
  Controller: RTrobot Servo Motor Controller (32 channel)
  Wiring: SoftwareSerial - Arduino pin 11 (RX) = controller TX,
                            Arduino pin 10 (TX) = controller RX

  Sends every channel (1-32) to pulse 1500, the servo's mechanical
  center/default position (500=0deg, 1500=90deg, 2500=180deg per the
  controller's pulse range).

  Note: this is each servo's own center point, not necessarily a uniform
  "neutral" pose for the whole hexapod, since horn mounting varies per
  servo - see the rest/stand/stretch poses for the actual tuned poses.
*/

#include <SoftwareSerial.h>
SoftwareSerial controllerSerial(11, 10); // RX, TX

const int NUM_CHANNELS = 32;
const int CENTER_PULSE = 1500;
const int RUN_TIME_MS  = 500; // T param - time to complete the move
const int DELAY_MS     = 500; // D param - pause after move

void setup() {
  Serial.begin(9600);             // USB debug monitor (optional)
  controllerSerial.begin(38400);  // must match controller's UART baud rate -- see esp32.ino's
                                   // Timing section for why this was raised from 9600
  delay(2000);                   // let the controller finish booting

  String cmd = "";
  for (int ch = 1; ch <= NUM_CHANNELS; ch++) {
    cmd += "#" + String(ch) + "P" + String(CENTER_PULSE);
  }
  cmd += "T" + String(RUN_TIME_MS) + "D" + String(DELAY_MS) + "\r\n";

  controllerSerial.print(cmd);
  Serial.println("Sent: all 32 channels -> 1500 (center)");
}

void loop() {
  // Echo any reply from the controller (e.g. "OK") to the Serial Monitor
  while (controllerSerial.available()) {
    char c = controllerSerial.read();
    Serial.write(c);
  }
}
