#include <Servo.h>
#include <ArduinoJson.h>

// Define servo objects for each servo motor
Servo waistServo;
Servo shoulderServo;
Servo elbowServo;
Servo wristRollServo;
Servo wristPitchServo;
Servo gripperServo;

// Define pins for each servo motor
int waistPin = 2;
int shoulderPin = 3;
int elbowPin = 4;
int wristRollPin = 5;
int wristPitchPin = 6;
int gripperPin = 7;

// Define initial positions for each servo motor
int initialWaistPos = 90;
int initialShoulderPos = 45;
int initialElbowPos = 120;
int initialWristRollPos = 0;
int initialWristPitchPos = 145;
int initialGripperPos = 140;

// Variables to store current positions
int waistPos = initialWaistPos;
int shoulderPos = initialShoulderPos;
int elbowPos = initialElbowPos;
int wristRollPos = initialWristRollPos;
int wristPitchPos = initialWristPitchPos;
int gripperPos = initialGripperPos;

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Attach each servo to its corresponding pin
  waistServo.attach(waistPin);
  shoulderServo.attach(shoulderPin);
  elbowServo.attach(elbowPin);
  wristRollServo.attach(wristRollPin);
  wristPitchServo.attach(wristPitchPin);
  gripperServo.attach(gripperPin);

  // Move each servo to its initial position
  moveServoToPosition(waistServo, initialWaistPos);
  moveServoToPosition(shoulderServo, initialShoulderPos);
  moveServoToPosition(elbowServo, initialElbowPos);
  moveServoToPosition(wristRollServo, initialWristRollPos);
  moveServoToPosition(wristPitchServo, initialWristPitchPos);
  moveServoToPosition(gripperServo, initialGripperPos);
}

void loop() {
  // Check if data is available
  if (Serial.available() > 0) {
    // Read the incoming JSON string
    String jsonString = Serial.readStringUntil('\n');
    
    // Parse JSON
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, jsonString);
    
    if (!error) {
      // Get new positions from JSON
      int newWaistPos = doc["waist"];
      int newShoulderPos = doc["shoulder"];
      int newElbowPos = doc["elbow"];
      int newWristRollPos = doc["wrist_roll"];
      int newWristPitchPos = doc["wrist_pitch"];
      int newGripperPos = doc["gripper"];
      
      // Update positions if they've changed
      if (newWaistPos != waistPos) {
        waistPos = newWaistPos;
        moveServoToPosition(waistServo, waistPos);
      }
      
      if (newShoulderPos != shoulderPos) {
        shoulderPos = newShoulderPos;
        moveServoToPosition(shoulderServo, shoulderPos);
      }
      
      if (newElbowPos != elbowPos) {
        elbowPos = newElbowPos;
        moveServoToPosition(elbowServo, elbowPos);
      }
      
      if (newWristRollPos != wristRollPos) {
        wristRollPos = newWristRollPos;
        moveServoToPosition(wristRollServo, wristRollPos);
      }
      
      if (newWristPitchPos != wristPitchPos) {
        wristPitchPos = newWristPitchPos;
        moveServoToPosition(wristPitchServo, wristPitchPos);
      }
      
      if (newGripperPos != gripperPos) {
        gripperPos = newGripperPos;
        moveServoToPosition(gripperServo, gripperPos);
      }
    }
  }
  
  // Small delay to prevent overwhelming the serial port
  delay(15);
}

void moveServoToPosition(Servo servo, int position) {
  // Move the servo to the specified position
  servo.write(position);
  delay(50); // Reduced delay for more responsive control
}

void returnToInitialPosition() {
  // Move each servo back to its initial position
  moveServoToPosition(waistServo, initialWaistPos);
  moveServoToPosition(shoulderServo, initialShoulderPos);
  moveServoToPosition(elbowServo, initialElbowPos);
  moveServoToPosition(wristRollServo, initialWristRollPos);
  moveServoToPosition(wristPitchServo, initialWristPitchPos);
  moveServoToPosition(gripperServo, initialGripperPos);
} 