#!/usr/bin/env python3

import cv2
import numpy as np
import os

# Path to the dataset
datasets = 'datasets'

# Create an LBPH face recognizer
model = cv2.face.LBPHFaceRecognizer_create()

# Load the Haar Cascade file
haar_file = 'haarcascade_frontalface_default.xml'
face_cascade = cv2.CascadeClassifier(haar_file)

# Initialize lists for faces and labels
faces = []
labels = []

# Get the names of all persons (subfolders in datasets)
subfolders = os.listdir(datasets)
for label, person_name in enumerate(subfolders):
    person_path = os.path.join(datasets, person_name)
    for filename in os.listdir(person_path):
        image_path = os.path.join(person_path, filename)
        image = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
        faces.append(image)
        labels.append(label)

# Train the model
model.train(faces, np.array(labels))
model.save('face_recognizer.yml')
