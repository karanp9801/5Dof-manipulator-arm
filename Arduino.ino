/*
  Manipulator5DOF.ino
  Controls a 5-DOF manipulator + gripper using 6x MG996 servos.

  Joints (mapping used here):
   - Base yaw       -> servoBase
   - Shoulder       -> servoShoulder
   - Elbow          -> servoElbow
   - Wrist pitch    -> servoWristP
   - Wrist roll     -> servoWristR (used as orientation)
   - Gripper        -> servoGripper

  */

#include <Servo.h>
#include <math.h>

// ------------------------ Pins ------------------------
const int PIN_BASE      = 2;
const int PIN_SHOULDER  = 3;
const int PIN_ELBOW     = 4;
const int PIN_WRIST_P   = 5;
const int PIN_WRIST_R   = 6;
const int PIN_GRIP      = 7;

Servo servoBase, servoShoulder, servoElbow, servoWristP, servoWristR, servoGripper;

// ------------------------ Geometry (mm) ------------------------
float L1 = 50.0;   // shoulder -> elbow
float L2 = 70.0;   // elbow -> wrist
float L3 = 35.0;   // wrist -> end-effector tip (tool length)

// ------------------------ Servo mapping & calibration ------------------------
int offsetBase     = 90;
int offsetShoulder = 90;
int offsetElbow    = 90;
int offsetWristP   = 90;
int offsetWristR   = 90;
int offsetGripper  = 10;

int invBase     = 1;
int invShoulder = -1;
int invElbow    = 1;
int invWristP   = 1;
int invWristR   = 1;
int invGripper  = 1;

const int SERVO_MIN = 0;
const int SERVO_MAX = 180;

float degToRad(float d){ return d * M_PI / 180.0; }
float radToDeg(float r){ return r * 180.0 / M_PI; }
int clampServo(int v){ if(v < SERVO_MIN) return SERVO_MIN; if(v > SERVO_MAX) return SERVO_MAX; return v; }

int jointToServo(float jointDeg, int offset, int invert){
  int s = round(offset + invert * jointDeg);
  return clampServo(s);
}

// ------------------------ Inverse Kinematics ------------------------
bool inverseKinematics(float tx, float ty, float tz, float wristPitchDeg,
                       float &baseDeg, float &shoulderDeg, float &elbowDeg, float &wristPitchDegOut)
{
  // 1) base rotation
  float baseRad = atan2(ty, tx);
  baseDeg = radToDeg(baseRad);

  // 2) planar projection for 2D arm (r,z)
  float r = sqrt(tx*tx + ty*ty);
  float xPlane = r;
  float zPlane = tz;

  // desired wrist pitch (radians)
  float wristPitchRad = degToRad(wristPitchDeg);

  // compute wrist joint target (point where L3 attaches)
  float wx = xPlane - L3 * cos(wristPitchRad);
  float wz = zPlane - L3 * sin(wristPitchRad);

  float D = sqrt(wx*wx + wz*wz);

  // reachability check
  if (D > (L1 + L2) || D < fabs(L1 - L2)) {
    return false;
  }

  // law of cosines for elbow
  float cosElbow = (wx*wx + wz*wz - L1*L1 - L2*L2) / (2.0 * L1 * L2);
  cosElbow = min(1.0f, max(-1.0f, cosElbow));
  float angleElbowRad = acos(cosElbow);

  // shoulder angle
  float k1 = L1 + L2 * cos(angleElbowRad);
  float k2 = L2 * sin(angleElbowRad);
  float angleShoulderRad = atan2(wz, wx) - atan2(k2, k1);

  shoulderDeg = radToDeg(angleShoulderRad);
  elbowDeg    = radToDeg(angleElbowRad);

  // simple wrist compensation: try to meet desired wrist pitch
  float totalArmPitchDeg = shoulderDeg + elbowDeg;
  wristPitchDegOut = wristPitchDeg - totalArmPitchDeg;

  return true;
}

// ------------------------ Motion & Gripper ------------------------
void moveToPose(float tx, float ty, float tz, float wristPitchDeg, int moveDelay=20){
  float baseDeg, shoulderDeg, elbowDeg, wristPitchDegOut;
  bool ok = inverseKinematics(tx, ty, tz, wristPitchDeg, baseDeg, shoulderDeg, elbowDeg, wristPitchDegOut);

  if(!ok){
    Serial.println("Target unreachable");
    return;
  }

  int sBase     = jointToServo(baseDeg, offsetBase, invBase);
  int sShoulder = jointToServo(shoulderDeg, offsetShoulder, invShoulder);
  int sElbow    = jointToServo(elbowDeg, offsetElbow, invElbow);
  int sWristP   = jointToServo(wristPitchDegOut, offsetWristP, invWristP);
  int sWristR   = jointToServo(0, offsetWristR, invWristR);

  // simple interpolation
  const int steps = 30;
  int currBase     = sBase;
  int currShoulder = sShoulder;
  int currElbow    = sElbow;
  int currWristP   = sWristP;
  int currWristR   = sWristR;

  for(int i=0;i<=steps;i++){
    float t = (float)i/steps;
    servoBase.write( round(currBase + t * (sBase - currBase)) );
    servoShoulder.write( round(currShoulder + t * (sShoulder - currShoulder)) );
    servoElbow.write( round(currElbow + t * (sElbow - currElbow)) );
    servoWristP.write( round(currWristP + t * (sWristP - currWristP)) );
    servoWristR.write( round(currWristR + t * (sWristR - currWristR)) );
    delay(moveDelay);
  }
}

void setGripper(bool open){
  int angle = open ? 10 + offsetGripper : 80 + offsetGripper;
  angle = clampServo(invGripper * angle);
  servoGripper.write(angle);
}

// ------------------------ Setup / Loop ------------------------
void setup(){
  Serial.begin(115200);
  servoBase.attach(PIN_BASE);
  servoShoulder.attach(PIN_SHOULDER);
  servoElbow.attach(PIN_ELBOW);
  servoWristP.attach(PIN_WRIST_P);
  servoWristR.attach(PIN_WRIST_R);
  servoGripper.attach(PIN_GRIP);

  // neutral positions
  servoBase.write(offsetBase);
  servoShoulder.write(offsetShoulder);
  servoElbow.write(offsetElbow);
  servoWristP.write(offsetWristP);
  servoWristR.write(offsetWristR);
  setGripper(true);

  delay(1000);
  Serial.println("Ready");
}

void loop(){
  // demo sequence (adjust positions to fit your build)
  moveToPose(80, 0, 10, 0);     // forward low
  delay(800);
  moveToPose(100, 30, 20, -20); // to the right
  delay(800);
  setGripper(false);
  delay(600);
  moveToPose(60, -40, 40, 10);
  delay(800);
  setGripper(true);
  delay(1200);
}
