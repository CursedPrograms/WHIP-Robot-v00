/*
  Hexapod - Rest -> Stand -> Stretch sequence
  Controller: RTrobot Servo Motor Controller (32 channel)
  Wiring: SoftwareSerial - Arduino pin 11 (RX) = controller TX,
                            Arduino pin 10 (TX) = controller RX

  Sends three poses in order, waiting for each move to finish before
  sending the next:
    1. Rest    - legs only (channels 1-9 + 24-32)
    2. Stretch - legs fully extended (channels 1-9 + 24-32)
    3. Stand   - all 32 channels
*/

#include <SoftwareSerial.h>
SoftwareSerial controllerSerial(11, 10); // RX, TX

const char REST_CMD[] =
  "#1P734#2P2188#3P2500#4P500#5P2500#6P2500#7P500#8P2500#9P500"
  "#24P2500#25P500#26P500#27P2500#28P500#29P500#30P2500#31P500#32P500"
  "T500D500\r\n";

const char STAND_UP_CMD[] =
  "#1P1383#2P890#3P1490#4P786#5P890#6P1253#7P500#8P890#9P1851"
  "#10P1500#11P1500#12P1500#13P1500#14P1500#15P1500#16P1500#17P1500#18P1500"
  "#19P1500#20P1500#21P1500#22P1500#23P1500"
  "#24P2500#25P1851#26P1747#27P2240#28P1435#29P1019#30P1851#31P1305#32P1591"
  "T500D500\r\n";

const char STRETCH_CMD[] =
  "#1P708#2P1643#3P630#4P526#5P1643#6P630#7P500#8P1643#9P630"
  "#24P2292#25P1201#26P2344#27P2240#28P1201#29P2344#30P2500#31P1201#32P2344"
  "T500D500\r\n";

// Move time (T) + post-move delay (D) from each command above, in ms -
// how long to wait before the controller is ready for the next instruction.
const int MOVE_SETTLE_MS = 500 + 500; // T500 + D500

void sendPose(const char* cmd, const char* label) {
  controllerSerial.print(cmd);
  Serial.print("Sent: ");
  Serial.println(label);
  delay(MOVE_SETTLE_MS + 200); // wait for move to finish, plus margin
}

void setup() {
  Serial.begin(9600);            // USB debug monitor (optional)
  controllerSerial.begin(9600);  // must match controller's UART baud rate
  delay(2000);                   // let the controller finish booting

  sendPose(REST_CMD, "rest pose");
  sendPose(STRETCH_CMD, "stretch pose");
  sendPose(STAND_UP_CMD, "stand up pose");
}

void loop() {
  // Echo any reply from the controller (e.g. "OK") to the Serial Monitor,
  // so you can confirm each pose was received and completed.
  while (controllerSerial.available()) {
    char c = controllerSerial.read();
    Serial.write(c);
  }
}
