
// ================= PCA9685 Setup ==================
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// Servo pulse limits
#define SERVOMIN  150
#define SERVOMAX  650

// Servo channels
#define BASE_SERVO      0
#define SHOULDER_SERVO  1
#define ELBOW_SERVO     2
#define WRIST_SERVO     3
#define WRIST_ROT_SERVO 4
#define GRIPPER_SERVO   5

// ================= TCS3200 Setup ==================
#define S0 8
#define S1 9
#define S2 10
#define S3 11
#define sensorOut 2

int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

// ================= Servo position tracking ==================
int servoPositions[6] = {90, 90, 90, 90, 90, 90}; // Current angles

// ================= Functions ==================
void setServoAngle(uint8_t n, int angle) {
  int pulselength = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(n, 0, pulselength);
}

void moveServoSmooth(uint8_t n, int startAngle, int endAngle, int stepDelay = 15) {
  if (startAngle < endAngle) {
    for (int a = startAngle; a <= endAngle; a++) {
      setServoAngle(n, a);
      delay(stepDelay);
    }
  } else {
    for (int a = startAngle; a >= endAngle; a--) {
      setServoAngle(n, a);
      delay(stepDelay);
    }
  }
  servoPositions[n] = endAngle;
}

int getCurrentAngle(uint8_t n) {
  return servoPositions[n];
}

void moveArmToInitial() {
  moveServoSmooth(BASE_SERVO, getCurrentAngle(BASE_SERVO), 90);
  moveServoSmooth(SHOULDER_SERVO, getCurrentAngle(SHOULDER_SERVO), 90);
  moveServoSmooth(ELBOW_SERVO, getCurrentAngle(ELBOW_SERVO), 90);
  moveServoSmooth(WRIST_SERVO, getCurrentAngle(WRIST_SERVO), 90);
  moveServoSmooth(WRIST_ROT_SERVO, getCurrentAngle(WRIST_ROT_SERVO), 90);
  moveServoSmooth(GRIPPER_SERVO, getCurrentAngle(GRIPPER_SERVO), 90);
}

// ================= Read color ==================
String readColor() {
  // RED
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redFrequency = pulseIn(sensorOut, LOW);

  // GREEN
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(sensorOut, LOW);

  // BLUE
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(sensorOut, LOW);

  Serial.print("R="); Serial.print(redFrequency);
  Serial.print(" G="); Serial.print(greenFrequency);
  Serial.print(" B="); Serial.println(blueFrequency);

  // ----- Color detection thresholds (adjust as needed) -----
  if (redFrequency < 40 && greenFrequency > 60 && blueFrequency > 60) {
    return "RED";
  }
  else if (blueFrequency < 35 && redFrequency > 60 && greenFrequency > 60) {
    return "BLUE";
  }

  return "OTHER";
}

// ================= Pick & Place RED ==================
void pickAndPlaceRed() {
  Serial.println(">>> RED Detected! Pick & place...");

  moveServoSmooth(WRIST_ROT_SERVO, getCurrentAngle(WRIST_ROT_SERVO), 140);
  moveServoSmooth(GRIPPER_SERVO, getCurrentAngle(GRIPPER_SERVO), 20);
  moveServoSmooth(SHOULDER_SERVO, getCurrentAngle(SHOULDER_SERVO), 90);
  moveServoSmooth(ELBOW_SERVO, getCurrentAngle(ELBOW_SERVO), 90);
  moveServoSmooth(WRIST_ROT_SERVO, getCurrentAngle(WRIST_ROT_SERVO), 140);

  // Drop position for RED
  moveServoSmooth(BASE_SERVO, getCurrentAngle(BASE_SERVO), 150);
  moveServoSmooth(SHOULDER_SERVO, getCurrentAngle(SHOULDER_SERVO), 50);
  moveServoSmooth(ELBOW_SERVO, getCurrentAngle(ELBOW_SERVO), 130);
  moveServoSmooth(GRIPPER_SERVO, getCurrentAngle(GRIPPER_SERVO), 90);
  moveServoSmooth(WRIST_ROT_SERVO, getCurrentAngle(WRIST_ROT_SERVO), 90);
  moveArmToInitial();
  Serial.println("RED Pick & Place Done!");
}

// ================= Pick & Place BLUE ==================
void pickAndPlaceBlue() {
  Serial.println(">>> BLUE Detected! Pick & place...");

  moveServoSmooth(WRIST_ROT_SERVO, getCurrentAngle(WRIST_ROT_SERVO), 140);
  moveServoSmooth(GRIPPER_SERVO, getCurrentAngle(GRIPPER_SERVO), 20);
  moveServoSmooth(SHOULDER_SERVO, getCurrentAngle(SHOULDER_SERVO), 90);
  moveServoSmooth(ELBOW_SERVO, getCurrentAngle(ELBOW_SERVO), 90);
  moveServoSmooth(WRIST_ROT_SERVO, getCurrentAngle(WRIST_ROT_SERVO), 140);

  // Drop position for BLUE (different from RED)
  moveServoSmooth(BASE_SERVO, getCurrentAngle(BASE_SERVO), 40);
  moveServoSmooth(SHOULDER_SERVO, getCurrentAngle(SHOULDER_SERVO), 50);
  moveServoSmooth(ELBOW_SERVO, getCurrentAngle(ELBOW_SERVO), 140);
  moveServoSmooth(GRIPPER_SERVO, getCurrentAngle(GRIPPER_SERVO), 90);
  moveServoSmooth(WRIST_ROT_SERVO, getCurrentAngle(WRIST_ROT_SERVO), 90);
  moveArmToInitial();
  Serial.println("BLUE Pick & Place Done!");
}

// ================= Setup ==================
void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(60);
  delay(10);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  moveArmToInitial();
  delay(1000);
  Serial.println("System Ready...");
}

// ================= Loop ==================
void loop() {
  String color = readColor();

  if (color == "RED") {
    pickAndPlaceRed();
    delay(3000);
  }
  else if (color == "BLUE") {
    pickAndPlaceBlue();
    delay(3000);
  }

  delay(500);
}
