/*
  ================================================================================
  HEXAPOD MASTER CONTROL ENGINE - KINEMATIC STANCE FIXED
  ================================================================================
  Features:
    - Highly symmetrical chassis translation
    - Pre-angled Femur & vertical Tibia standing offset calibration
    - 45-degree corner leg coordinate rotation matrix
    - Real-time ultrasonic obstacle avoidance

Right
    Resting pos for  Right back Coxas: 2000
    Max forward pan for Right back Coxas: 1200
     Max backward pan for Right back Coxas: 2500
    Resting pos for  Right Front Coxas: 1000
        Max forward pan for Right back Coxas: 1200
     Max backward pan for Right back Coxas: 2500

Right Middle Coxa Resting at 1500
Max Right Middle Forward Pan 1000 (If You Really Push and the front leg is also ponting forward it Lets say 1200)
Max Right Middle Backward Pan 2500 (If You Really Push and the back leg is also ponting backwards it Lets say 2300)


         - all Right  Femurs are pointing upwards at 2500
    - all Right  Femurs are bent 90 degrees at 1500
    - all Right  Femurs are pointing downwards 500
    
    - all Right  Tibias are straight at 500
    - all Right  Tibias are bent 90 degrees at 1500
    - all Right  Tibias are jackknife with femur 2500

Left

             Resting pos for  Left back Coxas: 1000
    Resting pos for  Left Front Coxas: 2000
          Max forward pan for Left back Coxas: 1700
     Max backward pan for Left back Coxas: 500
     Max forward pan for Left front Coxas: 2500
     Max backward pan for Left front Coxas: 1200

         - all Left  Femurs are pointing upwards at 500
    - all Left  Femurs are bent 90 degrees at 1500
    - all Left  Femurs are pointing downwards 2500

    - all Left  Tibias are straight at 2500
    - all Left  Tibias are bent 90 degrees at 1500
    - all Left  Tibias are jackknife with femur 500

    Left Middle Coxa Resting at 1500
Max Left Middle Forward Pan 2500 (If You Really Push and the front leg is also ponting forward it Lets say 2300)
Max Left Middle Backward Pan 1000 (If You Really Push and the back leg is also ponting backwards it Lets say 1200)






*/
/*
  ================================================================================
  HEXAPOD MASTER CONTROL ENGINE - FACTORY SPEC HARDWARE CALIBRATION
  ================================================================================
*/

#include <SoftwareSerial.h>
#include <Wire.h>

SoftwareSerial controllerSerial(11, 10); // (RX, TX)

const int TRIG_PIN = 7;
const int ECHO_PIN = 6;
const int DISTANCE_THRESHOLD_CM = 25; 

// ================================================================================
// 1. PHYSICAL GEOMETRY & STANCE CALIBRATION (CIGARETTE UNIT SCALE)
// ================================================================================
const float CIG_LENGTH = 84.0; 

const float L_TIBIA_VAL = CIG_LENGTH * 2.0; // 168mm
const float L_FEMUR_VAL = CIG_LENGTH * 1.0; // 84mm
const float L_COXA_LONG = CIG_LENGTH * 2.0; // 168mm (Corner Legs)
const float L_COXA_SHRT = CIG_LENGTH * 0.5; // 42mm  (Middle Legs)

struct LegConfig {
  int coxaCh; int femurCh; int tibiaCh;
  int legType;        // 0 = Back, 1 = Middle, 2 = Front
  bool isLeftSide;    // True if mounted on Left Side, False for Right Side
  float l_coxa; float l_femur; float l_tibia;
  float homeX; float homeY; float homeZ;
  float bodyAngleRad; 
};

const float RAD_45  = 45.0 * PI / 180.0;  
const float RAD_135 = 135.0 * PI / 180.0; 

// Base structural configuration array mapped precisely to physical channels
LegConfig Legs[6] = {
  // ------------------------------------ LEFT SIDE LIMBS ------------------------------------
  {1,  2,  3,  0, true,  L_COXA_LONG, L_FEMUR_VAL, L_TIBIA_VAL, 240.0, -50.0, -120.0, RAD_135}, // 0: Left Back
  {4,  5,  6,  1, true,  L_COXA_SHRT, L_FEMUR_VAL, L_TIBIA_VAL, 90.0,   0.0,  -120.0, 0.0},     // 1: Left Middle
  {7,  8,  9,  2, true,  L_COXA_LONG, L_FEMUR_VAL, L_TIBIA_VAL, 180.0,  60.0,  -120.0, RAD_45},  // 2: Left Front

  // ------------------------------------ RIGHT SIDE LIMBS ------------------------------------
  {24, 25, 26, 2, false, L_COXA_LONG, L_FEMUR_VAL, L_TIBIA_VAL, 180.0,  60.0,  -120.0, RAD_45},  // 3: Right Front
  {27, 28, 29, 1, false, L_COXA_SHRT, L_FEMUR_VAL, L_TIBIA_VAL, 90.0,   0.0,  -120.0, 0.0},     // 4: Right Middle
  {30, 31, 32, 0, false, L_COXA_LONG, L_FEMUR_VAL, L_TIBIA_VAL, 240.0, -50.0, -120.0, RAD_135}  // 5: Right Back
};

const float STEP_HEIGHT = 30.0; 
char globalCmdBuffer[650];      

enum GaitPattern  { GAIT_TRIPOD, GAIT_WAVE };
enum MoveDirection { MOVE_STOP, MOVE_FORWARD, MOVE_BACKWARD, MOVE_STRAFE_LEFT, MOVE_STRAFE_RIGHT, MOVE_TURN_LEFT, MOVE_TURN_RIGHT };

// ================================================================================
// 2. MATHEMATICAL INVERSE KINEMATICS MATRIX
// ================================================================================
struct LegAngles { float coxa; float femur; float tibia; };

LegAngles calculateLegIK(int legIdx, float x, float y, float z) {
  LegAngles angles;
  float lc = Legs[legIdx].l_coxa;
  float lf = Legs[legIdx].l_femur;
  float lt = Legs[legIdx].l_tibia;
  float bAngle = Legs[legIdx].bodyAngleRad;

  // Track rotational offset adjustments relative to the mounting angles
  float rotX = x * cos(bAngle) + y * sin(bAngle);
  float rotY = -x * sin(bAngle) + y * cos(bAngle);

  angles.coxa = atan2(rotY, rotX);
  
  float r = sqrt(rotX * rotX + rotY * rotY);
  float d = r - lc; 
  float targetDepth = abs(z);
  float s = sqrt(d * d + targetDepth * targetDepth);

  float cos_angleC = (lf * lf + lt * lt - s * s) / (2.0 * lf * lt);
  if(cos_angleC < -1.0) cos_angleC = -1.0; if(cos_angleC > 1.0) cos_angleC = 1.0; 
  float angleC = acos(cos_angleC);

  float cos_angleA = (lf * lf + s * s - lt * lt) / (2.0 * lf * s);
  if(cos_angleA < -1.0) cos_angleA = -1.0; if(cos_angleA > 1.0) cos_angleA = 1.0; 
  float angleA = acos(cos_angleA);

  // Return true standardized geometric delta outputs (0.0 rad = home aligned position)
  angles.femur = (atan2(targetDepth, d) + angleA) - HALF_PI; 
  angles.tibia = angleC - HALF_PI; 
  
  return angles;
}

// ================================================================================
// 3. HARDWARE SPECIFIC SERVO MICROSAMPLED CONVERSION ENGINE
// ================================================================================
void appendLegToBuffer(int legIdx, float x, float y, float z) {
  LegAngles target = calculateLegIK(legIdx, x, y, z);
  if (isnan(target.coxa) || isnan(target.femur) || isnan(target.tibia)) return;

  char temp[25];
  int pwmCoxa = 1500;
  int pwmFemur = 1500;
  int pwmTibia = 1500;

  bool isLeft = Legs[legIdx].isLeftSide;
  int type = Legs[legIdx].legType;

  // ------------------------- COXA JOINT STRUCTURAL CONVERSION -------------------------
  // Converts geometric angles to your precise unequal min/rest/max ranges
  if (!isLeft) { // Right Side Coxas
    if (type == 0) { // Right Back Coxa
      pwmCoxa = 2000 + round(target.coxa * 636.62);
      if(pwmCoxa < 1200) pwmCoxa = 1200; if(pwmCoxa > 2500) pwmCoxa = 2500;
    } else if (type == 2) { // Right Front Coxa
      pwmCoxa = 1000 + round(target.coxa * 636.62);
      if(pwmCoxa < 500) pwmCoxa = 500; if(pwmCoxa > 1800) pwmCoxa = 1800; // Scaled proportionally based on rest offsets
    } else { // Right Middle Coxa
      pwmCoxa = 1500 + round(target.coxa * 636.62);
      if(pwmCoxa < 1000) pwmCoxa = 1000; if(pwmCoxa > 2500) pwmCoxa = 2500;
    }
  } else { // Left Side Coxas
    if (type == 0) { // Left Back Coxa
      pwmCoxa = 1000 + round(target.coxa * 636.62);
      if(pwmCoxa < 500) pwmCoxa = 500; if(pwmCoxa > 1700) pwmCoxa = 1700;
    } else if (type == 2) { // Left Front Coxa
      pwmCoxa = 2000 + round(target.coxa * 636.62);
      if(pwmCoxa < 1200) pwmCoxa = 1200; if(pwmCoxa > 2500) pwmCoxa = 2500;
    } else { // Left Middle Coxa
      pwmCoxa = 1500 + round(target.coxa * 636.62);
      if(pwmCoxa < 1000) pwmCoxa = 1000; if(pwmCoxa > 2500) pwmCoxa = 2500;
    }
  }

  // ------------------------- FEMUR JOINT STRUCTURAL CONVERSION -------------------------
  // Mapped using: Right (2500 Up / 500 Down) vs Left (500 Up / 2500 Down)
  if (!isLeft) { 
    pwmFemur = 1500 + round(target.femur * 636.62); // Positive angle drives towards 2500 (Upwards)
  } else { 
    pwmFemur = 1500 - round(target.femur * 636.62); // Positive angle drives towards 500 (Upwards)
  }
  if(pwmFemur < 500) pwmFemur = 500; if(pwmFemur > 2500) pwmFemur = 2500;

  // ------------------------- TIBIA JOINT STRUCTURAL CONVERSION -------------------------
  // Mapped using: Right (500 Straight / 2500 Folded) vs Left (2500 Straight / 500 Folded)
  if (!isLeft) { 
    pwmTibia = 1500 + round(target.tibia * 636.62); // Positive flexing angle folds towards 2500
  } else { 
    pwmTibia = 1500 - round(target.tibia * 636.62); // Positive flexing angle folds towards 500
  }
  if(pwmTibia < 500) pwmTibia = 500; if(pwmTibia > 2500) pwmTibia = 2500;

  // Write finalized clean conversions directly into the serial transmission packet
  sprintf(temp, "#%dP%d#%dP%d#%dP%d", Legs[legIdx].coxaCh, pwmCoxa, Legs[legIdx].femurCh, pwmFemur, Legs[legIdx].tibiaCh, pwmTibia);
  strcat(globalCmdBuffer, temp);
}

void transmitBuffer(int durationMs) {
  char tail[20];
  sprintf(tail, "T%d\r\n", durationMs); 
  strcat(globalCmdBuffer, tail);
  controllerSerial.print(globalCmdBuffer); 
  delay(durationMs + 15);                  
}

// ================================================================================
// 4. SYMMETRICAL GAIT IMPLEMENTATION LOOP
// ================================================================================
void executeGait(MoveDirection dir, GaitPattern pattern, int phase, int executionTimeMs) {
  globalCmdBuffer[0] = '\0'; 
  float turnDir = 0; 
  float strideMag = 35.0; 

  if (dir == MOVE_TURN_LEFT)  turnDir = -1.0;
  if (dir == MOVE_TURN_RIGHT) turnDir =  1.0;

  for (int i = 0; i < 6; i++) {
    bool isSwing = false;
    if (pattern == GAIT_TRIPOD) {
      if (phase == 0) isSwing = (i == 0 || i == 2 || i == 4); 
      else            isSwing = (i == 1 || i == 3 || i == 5); 
    } else {
      const int waveSequence[6] = {2, 1, 0, 3, 4, 5}; 
      isSwing = (waveSequence[phase] == i);
    }

    float pushFactor = (pattern == GAIT_TRIPOD) ? 1.0 : 5.0;
    float targetX = Legs[i].homeX;
    float targetY = Legs[i].homeY;
    float targetZ = Legs[i].homeZ;

    if (dir != MOVE_STOP) {
      if (turnDir != 0) {
        float sideSign = (i < 3) ? 1.0 : -1.0; 
        if (isSwing) {
          targetY += (strideMag / 2.0) * turnDir * sideSign;
          targetZ += STEP_HEIGHT; 
        } else {
          targetY -= ((strideMag / 2.0) / pushFactor) * turnDir * sideSign;
        }
      } 
      else {
        float vecX = 0;
        if (dir == MOVE_FORWARD)  vecX =  strideMag;
        if (dir == MOVE_BACKWARD) vecX = -strideMag;

        if (isSwing) {
          targetX += vecX / 2.0;
          targetZ += STEP_HEIGHT; 
        } else {
          targetX -= (vecX / 2.0) / pushFactor;
        }
      }
    }
    appendLegToBuffer(i, targetX, targetY, targetZ);
  }
  transmitBuffer(executionTimeMs);
}

void moveRobot(MoveDirection dir, GaitPattern pattern, int stepsCount) {
  int totalPhases = (pattern == GAIT_TRIPOD) ? 2 : 6;
  int speedMs = (pattern == GAIT_TRIPOD) ? 280 : 160; 
  for (int step = 0; step < stepsCount; step++) {
    for (int phase = 0; phase < totalPhases; phase++) {
      executeGait(dir, pattern, phase, speedMs);
    }
  }
}

long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH, 20000); 
  return (duration == 0) ? 999 : (duration * 0.034 / 2);
}

void setup() {
  Serial.begin(9600);           
  controllerSerial.begin(9600); 
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  Wire.begin();

  delay(2000); 

  // Safely achieves your custom default mechanical alignment values
  globalCmdBuffer[0] = '\0';
  for(int i=0; i<6; i++) { appendLegToBuffer(i, Legs[i].homeX, Legs[i].homeY, Legs[i].homeZ); }
  transmitBuffer(1200); 
  delay(1000);
  Serial.println("Hexapod Operational Stance Confirmed. Calibration Vectors Aligned.");
}

void loop() {
  long distance = readDistanceCM();
  
  if (distance > 0 && distance < DISTANCE_THRESHOLD_CM) {
    Serial.println("[!] Obstacle Avoidance Routing Initiated...");
    executeGait(MOVE_STOP, GAIT_TRIPOD, 0, 300); 
    delay(200);
    moveRobot(MOVE_BACKWARD, GAIT_TRIPOD, 2);    
    executeGait(MOVE_STOP, GAIT_TRIPOD, 0, 300);
    moveRobot(MOVE_TURN_RIGHT, GAIT_TRIPOD, 3);   
    executeGait(MOVE_STOP, GAIT_TRIPOD, 0, 300);
  } 
  else {
    executeGait(MOVE_FORWARD, GAIT_TRIPOD, 0, 280);
    executeGait(MOVE_FORWARD, GAIT_TRIPOD, 1, 280);
  }

  while (controllerSerial.available()) { controllerSerial.read(); }
}
