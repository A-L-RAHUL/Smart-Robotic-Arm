import serial
import cv2
import mediapipe as mp
import json
import time

# Configuration
DEBUG = True
CAMERA_SOURCE = 0  # Use 0 for default webcam, or IP camera URL
WRITE_VIDEO = False

# Serial communication settings
SERIAL_PORT = 'COM4'  # Change to your Arduino port
BAUD_RATE = 9600

# Servo angle ranges
WAIST_MIN = 0
WAIST_MAX = 180
WAIST_MID = 90

SHOULDER_MIN = 0
SHOULDER_MAX = 180
SHOULDER_MID = 45

ELBOW_MIN = 0
ELBOW_MAX = 180
ELBOW_MID = 120

WRIST_ROLL_MIN = 0
WRIST_ROLL_MAX = 180
WRIST_ROLL_MID = 0

WRIST_PITCH_MIN = 135
WRIST_PITCH_MAX = 155
WRIST_PITCH_MID = 145

GRIPPER_MIN = 100
GRIPPER_MAX = 180
GRIPPER_MID = 140

# Hand tracking parameters
PALM_ANGLE_MIN = -50
PALM_ANGLE_MID = 20
PALM_SIZE_MIN = 0.1
PALM_SIZE_MAX = 0.3
WRIST_Y_MIN = 0.3
WRIST_Y_MAX = 0.9
FIST_THRESHOLD = 7

# Initialize MediaPipe
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_hands = mp.solutions.hands

# Initialize video capture
cap = cv2.VideoCapture(CAMERA_SOURCE)

# Video writer setup
if WRITE_VIDEO:
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter('output.avi', fourcc, 60.0, (640, 480))

# Helper functions
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

def map_range(x, in_min, in_max, out_min, out_max):
    return abs((x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min)

# Check if the hand is a fist
def is_fist(hand_landmarks, palm_size):
    # Calculate the distance between the wrist and each finger tip
    distance_sum = 0
    WRIST = hand_landmarks.landmark[0]
    for i in [7, 8, 11, 12, 15, 16, 19, 20]:
        distance_sum += ((WRIST.x - hand_landmarks.landmark[i].x)**2 + 
                         (WRIST.y - hand_landmarks.landmark[i].y)**2 + 
                         (WRIST.z - hand_landmarks.landmark[i].z)**2)**0.5
    return distance_sum/palm_size < FIST_THRESHOLD

# Convert hand landmarks to servo angles
def landmark_to_servo_angles(hand_landmarks):
    # Initialize with default positions
    servo_angles = {
        'waist': WAIST_MID,
        'shoulder': SHOULDER_MID,
        'elbow': ELBOW_MID,
        'wrist_roll': WRIST_ROLL_MID,
        'wrist_pitch': WRIST_PITCH_MID,
        'gripper': GRIPPER_MID
    }
    
    # Get key landmarks
    WRIST = hand_landmarks.landmark[0]
    INDEX_FINGER_MCP = hand_landmarks.landmark[5]
    THUMB_TIP = hand_landmarks.landmark[4]
    MIDDLE_TIP = hand_landmarks.landmark[12]
    
    # Calculate palm size
    palm_size = ((WRIST.x - INDEX_FINGER_MCP.x)**2 + 
                 (WRIST.y - INDEX_FINGER_MCP.y)**2 + 
                 (WRIST.z - INDEX_FINGER_MCP.z)**2)**0.5
    
    # Check if hand is a fist (for gripper control)
    if is_fist(hand_landmarks, palm_size):
        servo_angles['gripper'] = GRIPPER_MIN  # Close gripper
    else:
        servo_angles['gripper'] = GRIPPER_MAX  # Open gripper
    
    # Map thumb position to wrist roll
    thumb_y = clamp(THUMB_TIP.y, 0, 1)
    servo_angles['wrist_roll'] = map_range(thumb_y, 0, 1, WRIST_ROLL_MIN, WRIST_ROLL_MAX)
    
    # Map middle finger position to elbow and wrist pitch
    middle_y = clamp(MIDDLE_TIP.y, 0, 1)
    servo_angles['elbow'] = map_range(middle_y, 0, 1, ELBOW_MAX, ELBOW_MIN)
    servo_angles['wrist_pitch'] = map_range(middle_y, 0, 1, WRIST_PITCH_MIN, WRIST_PITCH_MAX)
    
    # Map wrist x position to waist rotation
    wrist_x = clamp(WRIST.x, 0, 1)
    servo_angles['waist'] = map_range(wrist_x, 0, 1, WAIST_MIN, WAIST_MAX)
    
    # Map wrist y position to shoulder
    wrist_y = clamp(WRIST.y, WRIST_Y_MIN, WRIST_Y_MAX)
    servo_angles['shoulder'] = map_range(wrist_y, WRIST_Y_MIN, WRIST_Y_MAX, SHOULDER_MIN, SHOULDER_MAX)
    
    # Convert to integers
    for key in servo_angles:
        servo_angles[key] = int(servo_angles[key])
    
    return servo_angles

# Connect to Arduino
if not DEBUG:
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to Arduino on {SERIAL_PORT}")
    except Exception as e:
        print(f"Failed to connect to Arduino: {e}")
        DEBUG = True

# Main loop
with mp_hands.Hands(model_complexity=0, min_detection_confidence=0.5, min_tracking_confidence=0.5) as hands:
    prev_servo_angles = None
    
    while cap.isOpened():
        success, image = cap.read()
        if not success:
            print("Ignoring empty camera frame.")
            continue
        
        # Flip the image horizontally for a later selfie-view display
        image = cv2.flip(image, 1)
        
        # Convert the BGR image to RGB
        image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # To improve performance, optionally mark the image as not writeable
        image_rgb.flags.writeable = False
        
        # Process the image and detect hands
        results = hands.process(image_rgb)
        
        # Draw hand annotations
        image_rgb.flags.writeable = True
        image = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        
        # Default servo angles
        servo_angles = {
            'waist': WAIST_MID,
            'shoulder': SHOULDER_MID,
            'elbow': ELBOW_MID,
            'wrist_roll': WRIST_ROLL_MID,
            'wrist_pitch': WRIST_PITCH_MID,
            'gripper': GRIPPER_MID
        }
        
        if results.multi_hand_landmarks:
            if len(results.multi_hand_landmarks) == 1:
                # Get hand landmarks
                hand_landmarks = results.multi_hand_landmarks[0]
                
                # Convert landmarks to servo angles
                servo_angles = landmark_to_servo_angles(hand_landmarks)
                
                # Send to Arduino if angles have changed
                if prev_servo_angles != servo_angles:
                    print("Servo angles:", servo_angles)
                    prev_servo_angles = servo_angles.copy()
                    
                    if not DEBUG:
                        try:
                            # Send as JSON
                            data = json.dumps(servo_angles) + '\n'
                            ser.write(data.encode())
                        except Exception as e:
                            print(f"Error sending data: {e}")
            else:
                print("More than one hand detected")
            
            # Draw hand landmarks
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS,
                    mp_drawing_styles.get_default_hand_landmarks_style(),
                    mp_drawing_styles.get_default_hand_connections_style())
        
        # Display servo angles on the image
        y_pos = 30
        for key, value in servo_angles.items():
            cv2.putText(image, f"{key}: {value}", (10, y_pos), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            y_pos += 30
        
        # Show the image
        cv2.imshow('Hand Tracking Robot Control', image)
        
        # Write to video if enabled
        if WRITE_VIDEO:
            out.write(image)
        
        # Break the loop if 'q' is pressed
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

# Clean up
cap.release()
if WRITE_VIDEO:
    out.release()
cv2.destroyAllWindows()

# Close serial connection
if not DEBUG and 'ser' in locals() and ser.is_open:
    ser.close()
    print("Serial connection closed") 