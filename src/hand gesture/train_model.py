# train_model.py

import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Conv2D, MaxPooling2D, Flatten, Dense, Dropout
from prepare_data import prepare_data

def create_model(input_shape, num_classes):
    model = Sequential([
        Conv2D(32, (3, 3), activation='relu', input_shape=input_shape),
        MaxPooling2D((2, 2)),
        Dropout(0.25),
        
        Conv2D(64, (3, 3), activation='relu'),
        MaxPooling2D((2, 2)),
        Dropout(0.25),
        
        Flatten(),
        Dense(128, activation='relu'),
        Dropout(0.5),
        Dense(num_classes, activation='softmax')
    ])
    
    model.compile(optimizer='adam', loss='categorical_crossentropy', metrics=['accuracy'])
    return model

if __name__ == "__main__":
    X_train, X_test, y_train, y_test, num_classes, class_names = prepare_data()
    
    if X_train is None:
        print("No data to train.")
        exit()
    
    model = create_model(X_train.shape[1:], num_classes)
    
    model.fit(X_train, y_train, epochs=10, validation_data=(X_test, y_test))
    
    model.save('gesture_model.h5')
    print("Model training complete.")
