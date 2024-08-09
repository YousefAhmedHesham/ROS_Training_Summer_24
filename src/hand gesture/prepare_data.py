# prepare_data.py

import os
import numpy as np
from sklearn.model_selection import train_test_split
from tensorflow.keras.utils import to_categorical
from tensorflow.keras.preprocessing.image import img_to_array, load_img

def load_data(dataset_path):
    images = []
    labels = []
    class_names = sorted(os.listdir(dataset_path))
    num_classes = len(class_names)
    
    for idx, class_name in enumerate(class_names):
        class_dir = os.path.join(dataset_path, class_name)
        for img_name in os.listdir(class_dir):
            img_path = os.path.join(class_dir, img_name)
            img = load_img(img_path, target_size=(100, 100))
            img_array = img_to_array(img) / 255.0
            images.append(img_array)
            labels.append(idx)
    
    images = np.array(images)
    labels = np.array(labels)
    labels = to_categorical(labels, num_classes)
    
    return images, labels, num_classes, class_names

def prepare_data():
    dataset_path = os.path.expanduser('~/trainingROS_ws/src/camera/src/hand gesture/gesture_dataset')
    images, labels, num_classes, class_names = load_data(dataset_path)
    
    if len(images) == 0:
        print("No images found in dataset.")
        return None, None, None, None, 0, []
    
    X_train, X_test, y_train, y_test = train_test_split(images, labels, test_size=0.2, random_state=42)
    
    print(f"Class names: {class_names}")
    print(f"Data preparation complete.")
    print(f"Number of classes: {num_classes}")
    print(f"Training samples: {len(X_train)}")
    print(f"Test samples: {len(X_test)}")
    
    return X_train, X_test, y_train, y_test, num_classes, class_names

if __name__ == "__main__":
    prepare_data()
