import pickle
import numpy as np
import tensorflow as tf
tf.python.control_flow_ops = tf
import csv
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np

    
# Initial Setup for Keras
from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten, Dropout, Lambda, Cropping2D
from keras.layers.convolutional import Convolution2D
from keras.layers.pooling import MaxPooling2D


# Import driving logs
lines= []
dataFolder = r'Track1Data6'
with open(dataFolder + '/driving_log.csv') as csvfile:
    reader = csv.reader(csvfile)
    for line in reader:
        lines.append(line)
    
# Import driving images    
images = []
measurements = []
for line in lines:
    source_path = line[0]
    filename = source_path.split('\\')[-1]
    current_path = dataFolder + '/IMG/' + filename
    image = mpimg.imread(current_path)
    images.append(image)
    measurements.append(line[3]) 
    
# Set training features and results
X_train = np.array(images)
y_train = np.array(measurements).astype('float')

# Flip the image to double sample size
X_train = np.concatenate((X_train, np.fliplr(X_train)), axis=0)
y_train = np.concatenate((y_train, -1.0*y_train), axis=0)


# TODO: Build the Final Test Neural Network in Keras Here
model = Sequential()
# Using NVIDIA pipeline
model.add(Cropping2D(cropping=((60,20),(1,1)), input_shape=(160,320,3)))
model.add(Lambda(lambda x: (x / 255.0) - 0.5)) # Normalize: output 80*320*3

model.add(Convolution2D(24, 5, 5)) #output 75*315*24
model.add(Activation('relu'))
model.add(MaxPooling2D((2, 2))) #output 35*157*24
model.add(Dropout(0.5))

model.add(Convolution2D(36, 5, 5)) #output 30*152*36
model.add(Activation('relu'))
model.add(MaxPooling2D((2, 2))) #output 15*76*36
# model.add(Dropout(0.5))

model.add(Convolution2D(48, 5, 5)) #output 10*71*48
model.add(Activation('relu'))
model.add(MaxPooling2D((2, 2))) #output 5*36*48

model.add(Convolution2D(64, 3, 3)) #output 2*33*64
model.add(Activation('relu'))
model.add(MaxPooling2D((2, 2))) #output 1*17*64
model.add(Dropout(0.5))

model.add(Flatten())  #output 1088
model.add(Dense(1000))
model.add(Activation('relu'))
model.add(Dense(100))
model.add(Activation('relu'))
model.add(Dense(50))
model.add(Activation('relu'))
model.add(Dense(10))
model.add(Activation('relu'))
model.add(Dense(1))

model.compile(loss='mse', optimizer='adam')
model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=3)

model.save('modelBData6+.h5')


    