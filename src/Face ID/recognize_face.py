#!/usr/bin/env python3

import cv2
import numpy as np

# Path to the Haar Cascade file
haar_file = 'haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(haar_file)

# Load the trained model
model = cv2.face.LBPHFaceRecognizer_create()
model.read('face_recognizer.yml')

# Names corresponding to labels
names = ['Yousef', 'Seif']  # Ensure the list matches the number of labels

# Start the webcam
webcam = cv2.VideoCapture(0)

# Set confidence threshold
confidence_threshold = 17  # Adjust based on your model's performance

while True:
    # Capture frame from webcam
    ret, frame = webcam.read()
    if not ret:
        print("Failed to capture image")
        break

    # Convert the frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect faces in the frame
    faces = face_cascade.detectMultiScale(gray, 1.3, 4)

    if len(faces) == 0:
        print("No faces detected")
    else:
        print(f"Detected {len(faces)} face(s)")

    # Draw rectangles around the faces and recognize them
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        face = gray[y:y + h, x:x + w]
        face_resize = cv2.resize(face, (600, 600))  # Resize to match the training size

        # Predict the face
        label, confidence = model.predict(face_resize)
        print(f"Predicted label: {label}, Confidence: {confidence}")

        # Determine if the face is recognized or unknown
        if confidence < confidence_threshold:
            if label < len(names):
                name = names[label]
            else:
                name = 'Unknown'
        else:
            name = 'Unknown'

        print(f"Recognized name: {name}")
        cv2.putText(frame, name, (x - 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # Show the frame
    cv2.imshow('Face Recognition', frame)

    # Break the loop if 'Esc' key is pressed
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Release the webcam and close windows
webcam.release()
cv2.destroyAllWindows()
