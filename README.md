# Face Recognition and Hand Gesture Recognition

This repository contains two Python scripts for face recognition and hand gesture recognition. Both scripts use OpenCV and other libraries to perform their respective tasks and interact with an Arduino for controlling digital pins based on detected gestures.

## Face Recognition

This script performs face recognition using the Local Binary Patterns Histograms (LBPH) algorithm with OpenCV.

### Prerequisites

- Python 3.x
- OpenCV
- A trained LBPH face recognizer model (`face_recognizer.yml`)
- Haar Cascade XML file (`haarcascade_frontalface_default.xml`)

### Installation

1. **Install Python dependencies:**

    ```bash
    pip install opencv-python numpy
    ```

2. **Prepare the Haar Cascade XML file:**

    Download `haarcascade_frontalface_default.xml` from the [OpenCV GitHub repository](https://github.com/opencv/opencv/tree/master/data/haarcascades) and place it in the project directory.

3. **Prepare the LBPH face recognizer model:**

    Ensure you have a trained model saved as `face_recognizer.yml` in the project directory.

### Usage

Run the face recognition script:

```bash
python3 face_recognition.py

The script will start the webcam, detect faces, and recognize them based on the trained model. It will display the recognized names on the video feed.

-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

# Hand Gesture Recognition

This Python script recognizes hand gestures using a pre-trained TensorFlow model and MediaPipe for hand detection. It also communicates with an Arduino to control digital pins based on detected gestures.

## Prerequisites

- Python 3.x
- OpenCV
- MediaPipe
- TensorFlow
- PySerial

## Installation

1. **Install Python dependencies:**

    ```bash
    pip install opencv-python mediapipe tensorflow pyserial numpy
    ```

2. **Prepare the TensorFlow model:**

    Ensure you have a trained TensorFlow model saved as `gesture_model.h5` in the project directory.

3. **Set up Arduino:**

    Connect your Arduino to `/dev/ttyACM0` (or adjust the port in the script) 

## Usage

Run the hand gesture recognition script:

```bash
python3 hand_gesture_recognition.py

The script will start the webcam, detect hand gestures, and classify them using the TensorFlow model. Based on the detected gesture, it will send commands to the Arduino via serial communication:

Open Hand Gesture: Sets pin 4 to HIGH.
Peace Gesture: Sets pin 5 to HIGH.
Unknown Gesture: Turns off both pins.

# Troubleshooting

Ensure the haarcascade_frontalface_default.xml and face_recognizer.yml files are in the correct paths.
Check the serial port in the hand gesture script; adjust if necessary.
Verify that all dependencies are correctly installed.
Confirm the Arduino is properly connected and the sketch is correctly uploaded.
