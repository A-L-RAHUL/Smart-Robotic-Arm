# Hand Tracking Robot Control

This project implements a web-based hand tracking system that controls a robotic arm using hand gestures. The system uses MediaPipe for hand tracking and communicates with an Arduino-controlled robotic arm.

## Features

- Real-time hand tracking using webcam
- Detection of thumb and middle finger positions
- Control of a 6-DOF robotic arm based on hand positions
- Modern web interface with connection status
- Serial port selection for Arduino communication

## Requirements

### Python Dependencies
- Flask
- OpenCV
- MediaPipe
- NumPy
- PySerial

### Hardware Requirements
- Webcam
- Arduino board (Uno or similar)
- 6x Servo motors for the robotic arm:
  - Waist servo (pin 2)
  - Shoulder servo (pin 3)
  - Elbow servo (pin 4)
  - Wrist roll servo (pin 5)
  - Wrist pitch servo (pin 6)
  - Gripper servo (pin 7)
- USB cable for Arduino connection

## Setup Instructions

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Arduino Setup:
   - Install the Arduino IDE
   - Install required libraries:
     - Servo.h (built-in)
     - ArduinoJson (install via Library Manager)
   - Connect servos to Arduino:
     - Waist servo to pin 2
     - Shoulder servo to pin 3
     - Elbow servo to pin 4
     - Wrist roll servo to pin 5
     - Wrist pitch servo to pin 6
     - Gripper servo to pin 7
   - Upload the `arduino_robot_arm.ino` sketch to your Arduino

3. Run the application:
```bash
python app.py
```

4. Open your web browser and navigate to:
```
http://localhost:5000
```

## Usage

1. Connect your Arduino to your computer
2. Select the correct COM port from the dropdown menu
3. Click "Connect" to establish communication with the Arduino
4. Click "Start Tracking" to begin hand tracking
5. Move your hand in front of the camera to control the robotic arm:
   - Thumb position (up/down) controls the wrist roll and gripper
   - Middle finger position (up/down) controls the elbow and wrist pitch

## How It Works

The system works by tracking your hand movements using MediaPipe and mapping them to the robotic arm's servos:

1. **Hand Tracking**: The webcam captures your hand movements, and MediaPipe detects the positions of your thumb and middle finger.

2. **Position Mapping**: 
   - Thumb position is mapped to wrist roll (0-180°) and gripper (180-100°)
   - Middle finger position is mapped to elbow (120-0°) and wrist pitch (155-135°)

3. **Arduino Control**: The Python application sends the finger positions to the Arduino via serial communication, which then controls the servos accordingly.

## Troubleshooting

- If the webcam is not detected, ensure it's properly connected and not in use by another application
- If the Arduino is not connecting, verify the correct COM port is selected
- If servos are not moving, check the wiring and power supply to the Arduino
- If the arm movements are erratic, try adjusting the mapping values in the Arduino code

## Safety Notes

- Ensure the robotic arm is properly secured before operation
- Keep fingers away from moving parts
- Disconnect power when making wiring changes
- Use appropriate power supply for the servos
- Start with slow movements to avoid sudden jerks that could damage the arm 