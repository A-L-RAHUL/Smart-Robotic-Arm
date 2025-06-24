#include <Servo.h>

// Define servo pins
const int GRIPPER_PIN = 2;
const int BASE_PIN = 3;
const int SHOULDER_PIN = 4;
const int ELBOW_PIN = 5;
const int WRIST_PIN = 6;
const int WRIST_ROTATE_PIN = 7;

// Create servo objects
Servo gripper;
Servo base;
Servo shoulder;
Servo elbow;
Servo wrist;
Servo wristRotate;

// Servo positions
int gripperPos = 90;    // 0: closed, 180: open
int basePos = 90;       // 0: left, 180: right
int shoulderPos = 90;   // 0: down, 180: up
int elbowPos = 90;      // 0: down, 180: up
int wristPos = 90;      // 0: down, 180: up
int wristRotatePos = 90;// 0: left, 180: right

void setup() {
    Serial.begin(9600);
    
    // Attach servos to pins
    gripper.attach(GRIPPER_PIN);
    base.attach(BASE_PIN);
    shoulder.attach(SHOULDER_PIN);
    elbow.attach(ELBOW_PIN);
    wrist.attach(WRIST_PIN);
    wristRotate.attach(WRIST_ROTATE_PIN);
    
    // Initialize servos to default positions
    moveAllServos();
}

void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        // Process command
        if (command == "grip_close") {
            gripperPos = 0;
        } else if (command == "grip_open") {
            gripperPos = 180;
        } else if (command == "move_forward") {
            // Move arm forward
            shoulderPos = constrain(shoulderPos + 10, 0, 180);
            elbowPos = constrain(elbowPos - 10, 0, 180);
        } else if (command == "move_backward") {
            // Move arm backward
            shoulderPos = constrain(shoulderPos - 10, 0, 180);
            elbowPos = constrain(elbowPos + 10, 0, 180);
        } else if (command == "move_left") {
            basePos = constrain(basePos - 10, 0, 180);
        } else if (command == "move_right") {
            basePos = constrain(basePos + 10, 0, 180);
        }
        
        // Update servo positions
        moveAllServos();
    }
}

void moveAllServos() {
    gripper.write(gripperPos);
    base.write(basePos);
    shoulder.write(shoulderPos);
    elbow.write(elbowPos);
    wrist.write(wristPos);
    wristRotate.write(wristRotatePos);
} 