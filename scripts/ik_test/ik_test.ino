/*
  HEXAPOD WALKING - SMOOTH GAIT
  - Initialize all to 1500
  - Wait 2 seconds
  - Start walking
  - Obstacle detection stops and returns to 1500
*/

#include <SoftwareSerial.h>

SoftwareSerial controller(11, 10);

const int TRIG_PIN = 7;
const int ECHO_PIN = 6;
const int MIN_DISTANCE = 60;

enum State { INIT, WAITING, WALK, OBSTACLE };
State state = INIT;

unsigned long timer = 0;
unsigned long gait_timer = 0;
long distance = 0;

// Neutral position (all at 1500)
int neutral[32] = {
  1500, 1500, 1500,  1500, 1500, 1500,  1500, 1500, 1500,  1500,
  1500, 1500, 1500,  1500, 1500, 1500,  1500, 1500, 1500,  1500,
  1500, 1500, 1500,  1500, 1500, 1500,  1500, 1500, 1500,  1500,
  1500, 1500
};

// Step 1: Tripod A lifted, Tripod B pushing
int step1[32] = {
  1800, 1500, 1500,  1300, 2500, 1800,  1800, 1500, 1500,  1500,
  1500, 1500, 1500,  1500, 1500, 1500,  1500, 1500, 1500,  1500,
  1500, 1500, 2500,  1800, 500, 1200,   1200, 1500, 1500,  1800,
  500, 1200
};

// Step 2: Tripod A pushing, Tripod B lifted
int step2[32] = {
  1300, 2500, 1500,  1800, 1500, 1500,  1300, 2500, 1500,  1500,
  1500, 1500, 1500,  1500, 1500, 1500,  1500, 1500, 1500,  1500,
  1500, 1200, 500,   1500, 1800, 500,   1200, 1200, 500,   1500,
  1200, 500
};

void sendPosition(int pos[32]) {
  String cmd = "";
  for (int i = 0; i < 32; i++) {
    cmd += "#" + String(i + 1) + "P" + String(pos[i]);
  }
  cmd += "T50D50\r\n";
  controller.print(cmd);
}

void interpolate(int from[32], int to[32], float blend) {
  int temp[32];
  for (int i = 0; i < 32; i++) {
    temp[i] = (int)(from[i] * (1.0 - blend) + to[i] * blend);
  }
  sendPosition(temp);
}

long readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return -1;
  return duration * 0.034 / 2;
}

void setup() {
  Serial.begin(9600);
  controller.begin(9600);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  delay(1000);
  
  // Initialize all servos to 1500
  sendPosition(neutral);
  state = WAITING;
  timer = millis();
  
  Serial.println("=== HEXAPOD ===");
  Serial.println("Initializing... waiting 2 seconds");
}

void loop() {
  // Check distance
  static unsigned long last_dist = 0;
  if (millis() - last_dist > 100) {
    last_dist = millis();
    distance = readDistance();
    if (distance > 0) {
      Serial.print("Distance: ");
      Serial.println(distance);
    }
  }
  
  unsigned long elapsed = millis() - timer;
  
  if (state == WAITING) {
    if (elapsed >= 2000) {
      state = WALK;
      gait_timer = millis();
      Serial.println("WALKING");
    }
  }
  
  else if (state == WALK) {
    // Check for obstacle
    if (distance > 0 && distance < MIN_DISTANCE) {
      state = OBSTACLE;
      timer = millis();
      Serial.println("OBSTACLE - STOPPING");
    } else {
      // Walk: alternate between step1 and step2
      float gait_phase = fmod((millis() - gait_timer) / 1200.0, 1.0);
      
      if (gait_phase < 0.5) {
        // Blend from neutral to step1, then step1 to step2
        float blend = gait_phase * 2.0;
        interpolate(neutral, step1, blend);
      } else {
        // Blend from step1 to step2, then step2 to neutral
        float blend = (gait_phase - 0.5) * 2.0;
        interpolate(step1, step2, blend);
      }
    }
  }
  
  else if (state == OBSTACLE) {
    // Return to neutral position
    sendPosition(neutral);
    
    // Check if clear
    if (distance < 0 || distance >= MIN_DISTANCE + 100) {
      state = WALK;
      gait_timer = millis();
      Serial.println("CLEAR - RESUMING");
    }
  }
  
  while (controller.available()) {
    Serial.write(controller.read());
  }
}
