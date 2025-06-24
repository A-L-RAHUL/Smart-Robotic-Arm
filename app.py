import cv2
import mediapipe as mp
import numpy as np
from flask import Flask, render_template, Response
from flask_socketio import SocketIO
import serial
import time

app = Flask(__name__)
socketio = SocketIO(app)

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_draw = mp.solutions.drawing_utils

# Initialize serial connection to Arduino
try:
    arduino = serial.Serial('COM3', 9600, timeout=1)
    time.sleep(2)  # Wait for Arduino to initialize
except:
    print("Could not connect to Arduino. Running in simulation mode.")
    arduino = None

def calculate_gesture(landmarks):
    """Calculate gesture based on hand landmarks"""
    # Get key points
    thumb_tip = landmarks[mp_hands.HandLandmark.THUMB_TIP]
    index_tip = landmarks[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    middle_tip = landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    ring_tip = landmarks[mp_hands.HandLandmark.RING_FINGER_TIP]
    pinky_tip = landmarks[mp_hands.HandLandmark.PINKY_TIP]
    
    # Calculate distances between fingertips
    thumb_index_dist = np.sqrt((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)
    thumb_middle_dist = np.sqrt((thumb_tip.x - middle_tip.x)**2 + (thumb_tip.y - middle_tip.y)**2)
    
    # Determine gesture
    if thumb_index_dist < 0.1 and thumb_middle_dist < 0.1:
        return "fist"
    elif thumb_index_dist > 0.2 and thumb_middle_dist > 0.2:
        return "open_hand"
    elif thumb_index_dist < 0.1 and thumb_middle_dist > 0.2:
        return "pointing"
    else:
        return "unknown"

def send_to_arduino(gesture):
    """Send gesture command to Arduino"""
    if arduino is None:
        print(f"Simulation: Sending gesture {gesture}")
        return
    
    commands = {
        "fist": "grip_close\n",
        "open_hand": "grip_open\n",
        "pointing": "move_forward\n"
    }
    
    if gesture in commands:
        try:
            arduino.write(commands[gesture].encode())
        except:
            print("Error sending command to Arduino")

def generate_frames():
    cap = cv2.VideoCapture(0)
    while True:
        success, frame = cap.read()
        if not success:
            break
        
        # Convert the BGR image to RGB
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        
        # Process the frame with MediaPipe
        results = hands.process(rgb_frame)
        
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # Draw landmarks on the frame
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Calculate gesture and send to Arduino
                gesture = calculate_gesture(hand_landmarks.landmark)
                send_to_arduino(gesture)
                
                # Display gesture on frame
                cv2.putText(frame, f"Gesture: {gesture}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Send gesture to web interface
                socketio.emit('gesture_update', {'gesture': gesture})
        
        # Convert the frame to JPEG format
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

@socketio.on('connect')
def connect():
    print('Client connected')

@socketio.on('disconnect')
def disconnect():
    print('Client disconnected')

if __name__ == '__main__':
    socketio.run(app, debug=True) 