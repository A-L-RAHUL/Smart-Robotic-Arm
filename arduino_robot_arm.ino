#include <Servo.h>

// Create servo objects
Servo waist;    // Base rotation
Servo shoulder; // Shoulder joint
Servo elbow;    // Elbow joint
Servo wristRoll; // Wrist rotation
Servo wristPitch; // Wrist up/down
Servo gripper;  // Gripper

// Store current positions
int pos[6] = {0, 90, 90, 0, 180, 180}; // Default positions: [waist, shoulder, elbow, wristRoll, wristPitch, gripper]

void setup() {
  Serial.begin(9600);
  
  // Attach servos to pins
  waist.attach(2);
  shoulder.attach(3);
  elbow.attach(4);
  wristRoll.attach(5);
  wristPitch.attach(6);
  gripper.attach(7);
  
  // Set initial positions
  waist.write(pos[0]);
  shoulder.write(pos[1]);
  elbow.write(pos[2]);
  wristRoll.write(pos[3]);
  wristPitch.write(pos[4]);
  gripper.write(pos[5]);
  
  Serial.println("ready");
}

void processCommand() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    int values[6];
    int index = 0;
    
    // Parse the input string
    int start = 0;
    while (index < 6 && start < input.length()) {
      int end = input.indexOf(' ', start);
      if (end == -1) end = input.length();
      values[index++] = input.substring(start, end).toInt();
      start = end + 1;
    }
    
    // Update positions with constraints
    if (index == 6) {
      // Waist: 0-180 degrees
      pos[0] = constrain(values[0], 0, 180);
      
      // Shoulder: Fixed at 90 degrees
      pos[1] = 90;
      
      // Elbow: 0-180 degrees
      pos[2] = constrain(values[2], 0, 180);
      
      // Wrist Roll: Fixed at 0 degrees
      pos[3] = 0;
      
      // Wrist Pitch: Fixed at 180 degrees
      pos[4] = 180;
      
      // Gripper: 100-180 degrees
      pos[5] = constrain(values[5], 100, 180);
      
      // Move servos
      waist.write(pos[0]);
      shoulder.write(pos[1]);
      elbow.write(pos[2]);
      wristRoll.write(pos[3]);
      wristPitch.write(pos[4]);
      gripper.write(pos[5]);
      
      // Send acknowledgment
      Serial.println("ok");
    }
  }
}

void loop() {
  processCommand();
  delay(10); // Small delay to prevent overwhelming the serial port
} 