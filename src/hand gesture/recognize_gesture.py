import cv2
import numpy as np
from tensorflow.keras.models import load_model
import mediapipe as mp
import serial

def predict_gesture(img, model, class_names, threshold=0.5):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = cv2.resize(img, (100, 100))
    img_array = np.expand_dims(img, axis=0) / 255.0
    predictions = model.predict(img_array)
    
    predicted_index = np.argmax(predictions)
    predicted_probability = predictions[0][predicted_index]
    
    if predicted_probability > threshold:
        predicted_class = class_names[predicted_index]
    else:
        predicted_class = 'Unknown Gesture'
    
    return predicted_class

if __name__ == "__main__":
    model = load_model('gesture_model.h5')
    class_names = ['open_hand', 'peace']  # Ensure these match your class names
    
    cap = cv2.VideoCapture(0)
    mp_hands = mp.solutions.hands
    hands = mp_hands.Hands()
    mp_draw = mp.solutions.drawing_utils

    ser = serial.Serial('/dev/ttyACM0', 9600)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(rgb_frame)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                x_min, y_min = np.min([(lm.x, lm.y) for lm in hand_landmarks.landmark], axis=0)
                x_max, y_max = np.max([(lm.x, lm.y) for lm in hand_landmarks.landmark], axis=0)
                h, w, _ = frame.shape
                x_min, y_min = int(x_min * w), int(y_min * h)
                x_max, y_max = int(x_max * w), int(y_max * h)
                hand_img = frame[y_min:y_max, x_min:x_max]

                if hand_img.size > 0:
                    gesture = predict_gesture(hand_img, model, class_names)
                    cv2.putText(frame, gesture, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                    if gesture != 'Unknown Gesture':
                        ser.write(f"{gesture}\n".encode())       # send to arduino

        cv2.imshow('Gesture Recognition', frame)
        
        if cv2.waitKey(1) & 0xFF == 27:  # ESC key to break
            break

    cap.release()
    cv2.destroyAllWindows()
