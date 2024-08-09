# Creating database
# It captures images and stores them in datasets
# folder under the folder name of sub_data
import cv2
import os

# Path to the Haar Cascade file for face detection
haar_file = 'haarcascades/haarcascade_frontalface_default.xml'

# Folder where all the face data will be stored
datasets = 'datasets'

# Sub folder within datasets to store images for a specific person
sub_data = 'vivek'  # You can change this to any name

# Create the path if it doesn't exist
path = os.path.join(datasets, sub_data)
if not os.path.isdir(path):
    os.makedirs(path)

# Define the size of the images
(width, height) = (130, 100)

# Load the Haar Cascade file
face_cascade = cv2.CascadeClassifier(haar_file)

# Start the webcam
webcam = cv2.VideoCapture(0)

# Initialize count for the number of images captured
count = 1

while count <= 60:
    # Capture a frame from the webcam
    (_, img) = webcam.read()
    
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Detect faces in the image
    faces = face_cascade.detectMultiScale(gray, 1.3, 4)
    
    for (x, y, w, h) in faces:
        # Draw a rectangle around the face
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)
        
        # Extract the face from the image
        face = gray[y:y + h, x:x + w]
        
        # Resize the face to the defined size
        face_resize = cv2.resize(face, (width, height))
        
        # Save the image in the defined path
        cv2.imwrite(f'{path}/{count}.png', face_resize)
    
        # Increment the count
        count += 1
    
    # Display the image with rectangles around detected faces
    cv2.imshow('OpenCV', img)
    
    # Break the loop if 'Esc' key is pressed
    key = cv2.waitKey(10)
    if key == 27:
        break

# Release the webcam and close all OpenCV windows
webcam.release()
cv2.destroyAllWindows()
