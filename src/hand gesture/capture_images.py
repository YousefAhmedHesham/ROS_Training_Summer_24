# capture_images_with_hand_tracking.py

import cv2
import mediapipe as mp
import os
import time

# Define paths for saving images
dataset_dir = os.path.expanduser('~/trainingROS_ws/src/camera/src/hand gesture/gesture_dataset')
folders = ['peace', 'open_hand']  # Add more gestures if needed

# Create directories if they don't exist
for folder in folders:
    os.makedirs(os.path.join(dataset_dir, folder), exist_ok=True)

# Initialize MediaPipe Hand module
mp_hands = mp.solutions.hands
hands = mp_hands.Hands(min_detection_confidence=0.8, min_tracking_confidence=0.8)
mp_drawing = mp.solutions.drawing_utils

def capture_images():
    cap = cv2.VideoCapture(0)
    gesture = None
    image_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Convert the frame to RGB for MediaPipe processing
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)
        
        # Draw hand landmarks on the frame
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
        
        # Display the frame
        cv2.imshow('Capture Gesture', frame)
        
        # Key press handling
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):  # 'q' to quit
            break
        elif key == ord('f'):  # 'f' to capture "fist"
            gesture = 'fist'
        elif key == ord('p'):  # 'p' to capture "peace"
            gesture = 'peace'
        elif key == ord('o'):  # 'o' to capture "open_hand"
            gesture = 'open_hand'
        
        if gesture and results.multi_hand_landmarks:
            image_count += 1
            img_path = os.path.join(dataset_dir, gesture, f'image_{image_count}.jpg')
            cv2.imwrite(img_path, frame)
            print(f"Captured {gesture} gesture. Image saved to {img_path}")
            time.sleep(1)  # Add a delay to avoid capturing the same frame multiple times
            gesture = None
        
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    capture_images()
