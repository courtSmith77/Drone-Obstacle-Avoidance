## https://www.analyticsvidhya.com/blog/2021/01/image-classification-using-convolutional-neural-networks-a-step-by-step-guide/

# from keras.models import Sequential
import tensorflow as tf
import keras

from keras import layers
from keras.utils import to_categorical
from sklearn.model_selection import train_test_split
import numpy as np
import os
import cv2
import random 
from numpy import *

path_data = os.path.join(".", "train")

CATEGORIES = ["up", "down", "left", "right"]
# CATEGORIES = ["up"]
IMG_SIZE = 200

# create training data set
training = []

def createTrainingData():
    # for category in CATEGORIES :
    #     folder = "Color arrows - " + category
    #     path = os.path.join(path_data, folder)
    #     class_num = CATEGORIES.index(category)
    for img in os.listdir(path_data):
        img_array = cv2.imread(os.path.join(path_data,img))
        new_array = cv2.resize(img_array, (IMG_SIZE, IMG_SIZE))

        # label
        category = img.split("_", 1)[0]
        class_num = CATEGORIES.index(category)
        training.append([new_array, class_num])

createTrainingData()

random.shuffle(training)

# assigning Labels and Features
X = []
y = []
for features, label in training:
    X.append(features)
    y.append(label)
X = np.array(X).reshape(-1, IMG_SIZE, IMG_SIZE, 3)

# Normalize X and COnvert Labels to Categorical Data
X = X.astype('float32')
X /= 255
Y = to_categorical(y,4)
print("Shape of Y")
print(shape(Y))

# Split X and Y for Use in CNN
X_train, X_test, y_train, y_test = train_test_split(X,Y,test_size=0.2,random_state=4)

print("Shape of X_train")
print(shape(X_train))
print("Shape of Y_train")
print(shape(y_train))
print("Shape of X_test")
print(shape(X_test))
print("Shape of Y_test")
print(shape(y_test))

# Define Compile and Train the CNN Model
batch_size = 16
nb_classes = 4
nb_epochs = 50
img_rows, img_columns = 200, 200
img_channel = 3
nb_filters = 32
nb_pool = 2
nb_conv = 3

model = keras.Sequential([
    # first conv block, activation included in cov2d
    layers.Conv2D(32, (3,3), input_shape=(200, 200, 3)),
    layers.Activation('relu'),
    layers.MaxPooling2D((2, 2)),
    # first conv block, activation included in cov2d
    layers.Conv2D(32, (3,3)),
    layers.Activation('relu'),
    layers.MaxPooling2D((2, 2)),
    # below this is the classifier
    layers.Flatten(),
    layers.Dense(128, activation=tf.nn.relu),
    layers.Dropout(0.5),
    layers.Dense(4,  activation=tf.nn.softmax)
])

print("Compiling")
model.compile(optimizer='adam',loss='categorical_crossentropy',metrics=['accuracy'])
print("Fitting")
model.fit(X_train, y_train, batch_size=batch_size, epochs=nb_epochs, verbose=1, validation_data=(X_test, y_test))

# Accuracy and Score of Model
print("Testing")
score = model.evaluate(X_test, y_test, verbose = 0)
print("Test Score: ", score[0])
print("Test accuracy: ", score[1])