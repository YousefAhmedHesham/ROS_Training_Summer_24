#!/usr/bin/env python3

import cv2
import os

# Path to Haar Cascade file
haar_file = 'haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(haar_file)

# Directory to store the dataset
datasets = 'datasets'
sub_data = 'Seif'  # Change this to the name of the person

# Create the main datasets directory if it does not exist
if not os.path.isdir(datasets):
    os.mkdir(datasets)

path = os.path.join(datasets, sub_data)
if not os.path.isdir(path):
    os.mkdir(path)

# Set the size of the images
(width, height) = (600, 600)

# Capture images from the webcam
webcam = cv2.VideoCapture(0)

# Initialize a counter for images
count = 1
while count <= 500:
    ret, frame = webcam.read()
    if not ret:
        print("Failed to capture image")
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    faces = face_cascade.detectMultiScale(gray, 1.3, 4)
    for (x, y, w, h) in faces:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        face = gray[y:y + h, x:x + w]
        face_resize = cv2.resize(face, (width, height))
        cv2.imwrite(f'{path}/{count}.png', face_resize)
        count += 1

    cv2.imshow('Creating Dataset', frame)
    if cv2.waitKey(10) == 27:
        break

webcam.release()
cv2.destroyAllWindows()
