#!/usr/bin/env python3

import cv2
import numpy as np

# Path to the Haar Cascade file
haar_file = 'haarcascades/haarcascade_frontalface_default.xml'

# Initialize the Haar Cascade face detector
face_cascade = cv2.CascadeClassifier(haar_file)

# Start the webcam
webcam = cv2.VideoCapture(0)

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
    
    # Draw rectangles around the faces
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
    
    # Show the frame with rectangles around faces
    cv2.imshow('Face Detection', frame)
    
    # Write 1 if faces are detected, otherwise write 0
    if len(faces) > 0:
        print(1)
    else:
        print(0)
    
    # Break the loop if 'Esc' key is pressed
    if cv2.waitKey(10) & 0xFF == 27:
        break

# Release the webcam and close windows
webcam.release()
cv2.destroyAllWindows()
