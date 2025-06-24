# Hand Tracking Robotic Arm Control

This project implements a hand tracking system that controls a 6-DOF robotic arm using MediaPipe for hand detection and tracking. The system maps hand movements to servo positions and communicates with an Arduino-controlled robotic arm.

## Features

- Real-time hand tracking using webcam
- Detection of hand gestures (open hand, fist)
- Control of a 6-DOF robotic arm based on hand movements
- Visual feedback with hand landmarks and servo positions
- Serial communication with Arduino

## Requirements

### Python Dependencies
- OpenCV
- MediaPipe
- NumPy
- PySerial
- ArduinoJson (for Arduino)

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
pip install opencv-python mediapipe numpy pyserial
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
   - Upload the `arduino_robot_arm_hand_tracking.ino` sketch to your Arduino

3. Configure the Python script:
   - Open `hand_tracking_robot.py`
   - Set `DEBUG = False` when ready to connect to Arduino
   - Set `SERIAL_PORT` to match your Arduino's port (e.g., 'COM4' for Windows)
   - Set `CAMERA_SOURCE` to your webcam index (usually 0) or IP camera URL

4. Run the application:
```bash
python hand_tracking_robot.py
```

## How to Control the Robotic Arm

The system maps your hand movements to the robotic arm's servos in the following way:

1. **Waist Rotation**: Controlled by the horizontal position of your wrist (left/right)
2. **Shoulder**: Controlled by the vertical position of your wrist (up/down)
3. **Elbow**: Controlled by the vertical position of your middle finger (up/down)
4. **Wrist Roll**: Controlled by the vertical position of your thumb (up/down)
5. **Wrist Pitch**: Controlled by the vertical position of your middle finger (up/down)
6. **Gripper**: Opens when your hand is open, closes when you make a fist

## Hand Gestures

- **Open Hand**: Gripper opens
- **Closed Fist**: Gripper closes
- **Move Hand Left/Right**: Controls waist rotation
- **Move Hand Up/Down**: Controls shoulder position
- **Move Middle Finger Up/Down**: Controls elbow and wrist pitch
- **Move Thumb Up/Down**: Controls wrist roll

## Troubleshooting

- If the webcam is not detected, check the `CAMERA_SOURCE` setting
- If the Arduino is not connecting, verify the correct `SERIAL_PORT` is selected
- If servos are not moving, check the wiring and power supply to the Arduino
- If the arm movements are erratic, try adjusting the mapping values in the Python script

## Safety Notes

- Ensure the robotic arm is properly secured before operation
- Keep fingers away from moving parts
- Disconnect power when making wiring changes
- Use appropriate power supply for the servos
- Start with slow movements to avoid sudden jerks that could damage the arm 