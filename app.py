from flask import Flask, render_template, Response, jsonify, request
import cv2
import mediapipe as mp
import numpy as np
import serial
import time
import os
import threading

# Suppress warnings and force CPU usage
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'  # Suppress TF logging
os.environ['MEDIAPIPE_DISABLE_GPU'] = '1'  # Force CPU usage
os.environ['TF_ENABLE_ONEDNN_OPTS'] = '0'  # Disable oneDNN

app = Flask(__name__)

# Global variables
camera = None
arduino = None
is_tracking = False
control_mode = "manual"  # Can be "manual" or "hand_tracking"
last_command_time = 0
debug_mode = True  # Enable debug mode
frame_buffer = None
buffer_lock = threading.Lock()
current_angles = [0, 90, 90, 0, 180, 180]  # Default angles for all motors
previous_angles = [0, 90, 90, 0, 180, 180]  # Store previous angles for smoothing
smoothing_factor = 0.3  # Lower value = more smoothing (0.0 to 1.0)
command_delay = 0.05  # Default delay between commands (50ms)
last_gripper_state = 180  # Track last gripper state for hysteresis

print("Initializing system...")

# Initialize MediaPipe Hands with lower complexity
print("Setting up hand tracking...")
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
    model_complexity=0  # Use simplest model for better performance
)
mp_draw = mp.solutions.drawing_utils

def debug_print(message):
    """Print debug messages if debug_mode is enabled"""
    if debug_mode:
        print(f"[DEBUG] {message}")

def connect_to_arduino():
    """Attempt to connect to Arduino with retries"""
    global arduino
    
    # Check available COM ports
    import serial.tools.list_ports
    ports = list(serial.tools.list_ports.comports())
    debug_print(f"Available COM ports: {[port.device for port in ports]}")
    
    # Try to find Arduino port
    arduino_port = None
    for port in ports:
        if "Arduino" in port.description or "CH340" in port.description or "USB Serial" in port.description:
            arduino_port = port.device
            debug_print(f"Found Arduino on port: {arduino_port}")
            break
    
    if not arduino_port:
        arduino_port = 'COM5'  # Default fallback
    
    max_attempts = 3
    print(f"\nAttempting to connect to Arduino on {arduino_port}...")
    
    for attempt in range(max_attempts):
        try:
            # Close existing connection if any
            if arduino and arduino.is_open:
                arduino.close()
                time.sleep(0.5)
            
            print(f"Connection attempt {attempt + 1}/{max_attempts}...")
            
            # Try to connect with higher timeout
            arduino = serial.Serial(arduino_port, 9600, timeout=2)
            time.sleep(2)  # Wait for Arduino to reset
            
            # Clear any pending data
            arduino.flushInput()
            arduino.flushOutput()
            
            # Test the connection
            for _ in range(3):  # Try the test command multiple times
                arduino.write(b"test\n")
                response = arduino.readline().decode().strip()
                debug_print(f"Arduino response: {response}")
                
                if response == "ready":
                    print(f"✓ Successfully connected to Arduino on {arduino_port}")
                    return True
                time.sleep(0.5)
            
            print(f"✗ Arduino not responding properly (attempt {attempt + 1})")
            arduino.close()
        except Exception as e:
            print(f"✗ Arduino connection error (attempt {attempt + 1}): {e}")
            if arduino:
                arduino.close()
                arduino = None
        time.sleep(1)
    
    print("✗ Failed to connect to Arduino after all attempts")
    return False

def init_camera():
    """Initialize the camera with error handling"""
    global camera
    try:
        print("\nInitializing camera...")
        if camera is not None:
            camera.release()
            time.sleep(0.5)
        
        camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # Try DirectShow first
        if not camera.isOpened():
            camera = cv2.VideoCapture(0)  # Fallback to default
        
        if not camera.isOpened():
            raise Exception("Could not open camera")
        
        # Set camera properties for better performance
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        camera.set(cv2.CAP_PROP_FPS, 30)
        camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize buffer size
        
        print("✓ Camera initialized successfully")
        return True
    except Exception as e:
        print(f"✗ Camera error: {str(e)}")
        return False

def process_hand_landmarks(landmarks):
    """Convert hand landmarks to servo angles with specific control scheme"""
    global previous_angles, current_angles, last_gripper_state
    
    try:
        # Get key points
        wrist = landmarks.landmark[0]
        thumb_tip = landmarks.landmark[4]
        index_tip = landmarks.landmark[8]
        middle_tip = landmarks.landmark[12]
        ring_tip = landmarks.landmark[16]
        pinky_tip = landmarks.landmark[20]

        # Calculate angles with specific control scheme
        # Waist rotation based on left/right hand movement
        waist = int(np.interp(wrist.x, [0.2, 0.8], [0, 180]))
        
        # Shoulder is fixed at 90 degrees
        shoulder = 90
        
        # Elbow based on up/down hand movement
        elbow = int(np.interp(wrist.y, [0.2, 0.8], [0, 180]))
        
        # Wrist roll is fixed at 0 degrees
        wrist_roll = 0
        
        # Wrist pitch is fixed at 180 degrees
        wrist_pitch = 180
        
        # Improved gripper control using multiple finger distances
        # Calculate distances between thumb and all fingertips
        thumb_to_index = ((thumb_tip.x - index_tip.x)**2 + 
                          (thumb_tip.y - index_tip.y)**2 + 
                          (thumb_tip.z - index_tip.z)**2)**0.5
        thumb_to_middle = ((thumb_tip.x - middle_tip.x)**2 + 
                           (thumb_tip.y - middle_tip.y)**2 + 
                           (thumb_tip.z - middle_tip.z)**2)**0.5
        thumb_to_ring = ((thumb_tip.x - ring_tip.x)**2 + 
                         (thumb_tip.y - ring_tip.y)**2 + 
                         (thumb_tip.z - ring_tip.z)**2)**0.5
        
        # Use the average of the three distances for more stable gripper control
        avg_finger_distance = (thumb_to_index + thumb_to_middle + thumb_to_ring) / 3
        
        # Enhanced gripper control with better thresholds and hysteresis
        raw_gripper = int(np.interp(avg_finger_distance, [0.02, 0.15], [100, 180]))
        
        # Apply hysteresis to prevent jitter
        if raw_gripper < 120:  # Trying to close
            if last_gripper_state > 150:  # Was previously open
                gripper = last_gripper_state  # Keep previous state
            else:
                gripper = raw_gripper
        elif raw_gripper > 160:  # Trying to open
            if last_gripper_state < 130:  # Was previously closed
                gripper = last_gripper_state  # Keep previous state
            else:
                gripper = raw_gripper
        else:
            gripper = raw_gripper
        
        # Update last gripper state
        last_gripper_state = gripper

        # Constrain all angles to valid ranges
        new_angles = [
            max(0, min(180, waist)),
            shoulder,  # Always 90 degrees
            max(0, min(180, elbow)),
            wrist_roll,  # Fixed at 0 degrees
            wrist_pitch,  # Fixed at 180 degrees
            max(100, min(180, gripper))
        ]
        
        # Apply smoothing to reduce jitter
        smoothed_angles = []
        for i in range(len(new_angles)):
            # Apply exponential smoothing
            smoothed = int(smoothing_factor * new_angles[i] + (1 - smoothing_factor) * previous_angles[i])
            smoothed_angles.append(smoothed)
        
        # Update previous and current angles
        previous_angles = smoothed_angles
        current_angles = smoothed_angles
        
        debug_print(f"Raw angles: {new_angles}")
        debug_print(f"Smoothed angles: {smoothed_angles}")
        debug_print(f"Gripper distance: {avg_finger_distance:.3f}, Raw: {raw_gripper}, Final: {gripper}")
        return smoothed_angles
    except Exception as e:
        print(f"Error processing landmarks: {e}")
        return [0, 90, 90, 0, 180, 180]  # Return safe default angles

def send_to_arduino(angles):
    """Send angles to Arduino with error handling"""
    global arduino, last_command_time, command_delay
    
    current_time = time.time()
    if current_time - last_command_time < command_delay:  # Use adjustable delay
        return False
        
    if arduino and arduino.is_open:
        try:
            command = f"{angles[0]} {angles[1]} {angles[2]} {angles[3]} {angles[4]} {angles[5]}\n"
            debug_print(f"Sending command: {command.strip()}")
            arduino.write(command.encode())
            arduino.flush()
            last_command_time = current_time
            return True
        except Exception as e:
            print(f"Arduino write error: {e}")
            connect_to_arduino()
            return False
    else:
        connect_to_arduino()
        return False

def process_frame():
    """Process camera frames in a separate thread"""
    global frame_buffer, camera, is_tracking, control_mode
    
    while True:
        if camera is None or not camera.isOpened():
            if not init_camera():
                time.sleep(1)
                continue
        
        try:
            success, frame = camera.read()
            if not success:
                print("Failed to read camera frame")
                init_camera()
                time.sleep(0.1)
                continue
            
            # Flip the frame horizontally for selfie view
            frame = cv2.flip(frame, 1)
            
            # Only process hand tracking if in hand_tracking mode
            if control_mode == "hand_tracking" and is_tracking:
                # Convert BGR to RGB
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Process the frame and detect hands
                results = hands.process(frame_rgb)
                
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        # Draw landmarks
                        mp_draw.draw_landmarks(
                            frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                        
                        # Get servo angles and send to Arduino
                        angles = process_hand_landmarks(hand_landmarks)
                        success = send_to_arduino(angles)
                        
                        # Display angles and status on frame
                        status_color = (0, 255, 0) if success else (0, 0, 255)
                        cv2.putText(frame, f"Angles: {angles}", (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
            
            # Add tracking status to frame
            status = f"Mode: {control_mode.upper()}"
            cv2.putText(frame, status, (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            if control_mode == "hand_tracking":
                tracking_status = "Tracking ON" if is_tracking else "Tracking OFF"
                cv2.putText(frame, tracking_status, (10, 90), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Update frame buffer
            with buffer_lock:
                frame_buffer = frame.copy()
            
            # Add a small delay to reduce CPU usage
            time.sleep(0.01)
            
        except Exception as e:
            print(f"Error in process_frame: {e}")
            time.sleep(0.1)

def generate_frames():
    """Generate frames for the web interface"""
    while True:
        try:
            # Wait for frame buffer to be populated
            if frame_buffer is None:
                time.sleep(0.1)
                continue
            
            # Get the latest frame
            with buffer_lock:
                frame = frame_buffer.copy()
            
            # Convert frame to jpg
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            # Add a small delay to reduce CPU usage
            time.sleep(0.033)  # ~30 FPS
            
        except Exception as e:
            print(f"Error in generate_frames: {e}")
            time.sleep(0.1)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/toggle_tracking', methods=['POST'])
def toggle_tracking():
    global is_tracking, control_mode
    
    # If switching to hand tracking mode
    if control_mode != "hand_tracking":
        control_mode = "hand_tracking"
        is_tracking = True
    else:
        # Toggle tracking within hand tracking mode
        is_tracking = not is_tracking
    
    debug_print(f"Control mode: {control_mode}, Tracking: {'enabled' if is_tracking else 'disabled'}")
    return jsonify({
        'tracking': is_tracking,
        'control_mode': control_mode
    })

@app.route('/set_control_mode', methods=['POST'])
def set_control_mode():
    """Set the control mode (manual or hand_tracking)"""
    global control_mode, is_tracking
    
    try:
        data = request.get_json()
        if 'mode' in data:
            new_mode = data['mode']
            
            # Validate mode
            if new_mode not in ["manual", "hand_tracking"]:
                return jsonify({'status': 'error', 'message': 'Invalid mode'}), 400
            
            # Set the mode
            control_mode = new_mode
            
            # If switching to manual mode, disable tracking
            if control_mode == "manual":
                is_tracking = False
            
            debug_print(f"Control mode set to: {control_mode}")
            return jsonify({
                'status': 'success',
                'control_mode': control_mode,
                'tracking': is_tracking
            })
        else:
            return jsonify({'status': 'error', 'message': 'Missing mode parameter'}), 400
    except Exception as e:
        debug_print(f"Error setting control mode: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/return_home', methods=['POST'])
def return_home():
    if not arduino or not arduino.is_open:
        if not connect_to_arduino():
            return jsonify({'status': 'error', 'message': 'Could not connect to Arduino'}), 500
    
    try:
        # Send home position command (neutral position)
        command = "90 90 90 90 155 180\n"
        debug_print(f"Sending home command: {command.strip()}")
        arduino.write(command.encode())
        arduino.flush()
        time.sleep(0.1)
        return jsonify({'status': 'success'})
    except Exception as e:
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/check_connection', methods=['GET'])
def check_connection():
    """Check if Arduino is connected and responding"""
    if arduino and arduino.is_open:
        try:
            # Send a test command
            arduino.write(b"test\n")
            arduino.flush()
            
            # Wait for response with timeout
            start_time = time.time()
            while time.time() - start_time < 1.0:  # 1 second timeout
                if arduino.in_waiting:
                    response = arduino.readline().decode().strip()
                    if response == "ready":
                        return jsonify({'connected': True})
                time.sleep(0.01)
            
            # If we get here, no valid response was received
            return jsonify({'connected': False})
        except Exception as e:
            debug_print(f"Connection check error: {e}")
            return jsonify({'connected': False})
    else:
        return jsonify({'connected': False})

@app.route('/get_angles', methods=['GET'])
def get_angles():
    """Return the current angles of all motors"""
    global current_angles
    return jsonify({'angles': current_angles})

@app.route('/update_sensitivity', methods=['POST'])
def update_sensitivity():
    """Update the smoothing factor for hand tracking"""
    global smoothing_factor
    
    try:
        data = request.get_json()
        if 'smoothing_factor' in data:
            # Constrain smoothing factor between 0.1 and 0.9
            new_factor = max(0.1, min(0.9, float(data['smoothing_factor'])))
            smoothing_factor = new_factor
            debug_print(f"Sensitivity updated: {smoothing_factor}")
            return jsonify({'status': 'success', 'smoothing_factor': smoothing_factor})
        else:
            return jsonify({'status': 'error', 'message': 'Missing smoothing_factor parameter'}), 400
    except Exception as e:
        debug_print(f"Error updating sensitivity: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/update_delay', methods=['POST'])
def update_delay():
    """Update the delay between commands"""
    global command_delay
    
    try:
        data = request.get_json()
        if 'delay' in data:
            # Constrain delay between 0.01 and 0.5 seconds
            new_delay = max(0.01, min(0.5, float(data['delay'])))
            command_delay = new_delay
            debug_print(f"Command delay updated: {command_delay}")
            return jsonify({'status': 'success', 'delay': command_delay})
        else:
            return jsonify({'status': 'error', 'message': 'Missing delay parameter'}), 400
    except Exception as e:
        debug_print(f"Error updating delay: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

@app.route('/update_angles', methods=['POST'])
def update_angles():
    """Update motor angles from manual control"""
    global control_mode
    
    # Ensure we're in manual mode
    if control_mode != "manual":
        control_mode = "manual"
    
    try:
        data = request.get_json()
        if 'angles' in data:
            angles = data['angles']
            # Ensure all angles are within valid ranges
            angles = [
                max(0, min(180, angles[0])),  # Waist
                90,  # Fixed shoulder
                max(0, min(180, angles[2])),  # Elbow
                max(0, min(180, angles[3])),  # Wrist roll
                max(0, min(180, angles[4])),  # Wrist pitch
                max(100, min(180, angles[5]))  # Gripper
            ]
            
            # Send to Arduino
            success = send_to_arduino(angles)
            if success:
                # Update current angles
                global current_angles
                current_angles = angles
                return jsonify({'status': 'success', 'angles': angles})
            else:
                return jsonify({'status': 'error', 'message': 'Failed to send to Arduino'}), 500
        else:
            return jsonify({'status': 'error', 'message': 'Missing angles parameter'}), 400
    except Exception as e:
        debug_print(f"Error updating angles: {e}")
        return jsonify({'status': 'error', 'message': str(e)}), 500

if __name__ == '__main__':
    print("\nInitializing system...")
    
    # Initialize camera first
    if not init_camera():
        print("Warning: Camera initialization failed")
    
    # Try to connect to Arduino
    if not connect_to_arduino():
        print("Warning: Arduino connection failed")
    
    # Start frame processing thread
    frame_thread = threading.Thread(target=process_frame, daemon=True)
    frame_thread.start()
    
    print("\nSystem initialization complete!")
    print("Access the control interface at: http://127.0.0.1:5000")
    
    # Run Flask app without debug mode for better performance
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True) 