/*
  HEXAPOD WALKING - 4-STEP TRIPOD GAIT
  Uses actual animated positions
*/

#include <SoftwareSerial.h>

SoftwareSerial controller(11, 10);

const int TRIG_PIN = 7;
const int ECHO_PIN = 6;
const int MIN_DISTANCE = 30;

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

// Step 1: Tripod A lifted and forward
int step1[32] = {
  1300, 2500, 1300,  1500, 1500, 1500,  1300, 2500, 1300,  1500,
  1500, 1500, 1500,  1500, 1500, 1500,  1500, 1500, 1500,  1500,
  1500, 1500, 1500,  1500, 1500, 1500,  1700, 500, 1700,   1500,
  1500, 1500
};

// Step 2: Hold position
int step2[32] = {
  1300, 1500, 1500,  1500, 1500, 1500,  1300, 1500, 1500,  1500,
  1500, 1500, 1500,  1500, 1500, 1500,  1500, 1500, 1500,  1500,
  1500, 1500, 1500,  1500, 1500, 1500,  1700, 1500, 1700,   1500,
  1700, 1500
};

// Step 3: Tripod B lifted and forward
int step3[32] = {
  1500, 1500, 1500,  1300, 2500, 1300,  1500, 1500, 1500,  1500,
  1500, 1500, 1500,  1500, 1500, 1500,  1500, 1500, 1500,  1500,
  1500, 1500, 1500,  1700, 500, 1700,   1500, 1500, 1500,  1700,
  1500, 1700
};

// Step 4: Hold position
int step4[32] = {
  1500, 1500, 1500,  1300, 1500, 1500,  1500, 1500, 1500,  1500,
  1500, 1500, 1500,  1500, 1500, 1500,  1500, 1500, 1500,  1500,
  1500, 1500, 1500,  1700, 1500, 1500,  1500, 1500, 1500,  1700,
  1500, 1500
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
  
  // Initialize all servos to neutral
  sendPosition(neutral);
  state = WAITING;
  timer = millis();
  
  Serial.println("=== HEXAPOD 4-STEP GAIT ===");
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
      // 4-step gait cycle: ~1200ms total
      float gait_phase = fmod((millis() - gait_timer) / 1200.0, 1.0);
      
      if (gait_phase < 0.25) {
        // Step 1: neutral to step1
        float blend = gait_phase * 4.0;
        interpolate(neutral, step1, blend);
      } 
      else if (gait_phase < 0.5) {
        // Step 2: step1 to step2 (hold)
        float blend = (gait_phase - 0.25) * 4.0;
        interpolate(step1, step2, blend);
      }
      else if (gait_phase < 0.75) {
        // Step 3: step2 to step3
        float blend = (gait_phase - 0.5) * 4.0;
        interpolate(step2, step3, blend);
      }
      else {
        // Step 4: step3 to step4 (hold), then back to neutral
        float blend = (gait_phase - 0.75) * 4.0;
        interpolate(step3, step4, blend);
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
