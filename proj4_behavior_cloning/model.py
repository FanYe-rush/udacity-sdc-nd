# https://www.dropbox.com/sh/d26zn0qpht2mmok/AAArZKTNxQ8hZSkVjIcX1lU9a?dl=1 (recovery)
# https://www.dropbox.com/sh/0n2aob505viagq1/AADi1zt2aXvSBtrn68RE5oy1a?dl=1 (sharp1)
# https://www.dropbox.com/sh/jopqkp1he1lwhgt/AAAB9ULD875E6gUhs5q_Z293a?dl=1 (sharp2)

import numpy as np
import tensorflow as tf
import cv2
import csv
import matplotlib.pyplot as plt
import os


def extractImgAndMeasure(data_paths):
    imgs = []
    measurements = []

    for data_path in data_paths:
        center_images = []
        center_measurements = []
        left_images = []
        left_measurements = []
        right_images = []
        right_measurements = []

        lines = []
        with open(data_path + "driving_log.csv") as csvfile:
            reader = csv.reader(csvfile)
            for line in reader:
                if line[0] == "center":
                    continue
                lines.append(line)

        for line in lines:
            center_source_path = line[0]
            left_source_path = line[1]
            right_source_path = line[2]

            # While cleaning up collected data, I only deteled the center images of some unwanted segments,
            # add a flag here to make sure the left/right images are only included if the corresponding center images
            # are still present
            center_found = False

            img = getPathForItem(center_source_path, data_path)
            if (img is not None):
                center_found = True
                center_images.append(img)
                center_measurements.append(float(line[3]))

            img = getPathForItem(left_source_path, data_path)
            if (img is not None) and center_found:
                adjust = 0.25
                # For relatively large steering angle, the adjustment for left side should be bigger as well
                if (float(line[3]) > 0.2):
                    adjust = 0.3

                left_images.append(img)
                left_measurements.append(float(line[3])+adjust)

            img = getPathForItem(right_source_path, data_path)
            if (img is not None) and center_found:
                adjust = 0.25
                # For relatively large steering angle, the adjustment for right side should be smaller
                if (float(line[3]) > 0.2):
                    adjust = 0.2

                right_images.append(img)
                right_measurements.append(float(line[3])-adjust)

        center_images.extend(left_images)
        center_images.extend(right_images)

        center_measurements.extend(left_measurements)
        center_measurements.extend(right_measurements)

        # Augment the data by flipping images left to right, and negate the steering angle
        flipped = np.fliplr(center_images)
        flipped_measurements = [-x for x in center_measurements]

        center_images.extend(flipped)
        center_measurements.extend(flipped_measurements)

        imgs.extend(center_images)
        measurements.extend(center_measurements)

    x_train = np.array(imgs)
    y_train = np.array(measurements)

    return x_train, y_train

def getPathForItem(source_path, data_path):
    filename = source_path.split('/')[-1]
    path = data_path + 'IMG/' + filename
    if os.path.exists(path):
        return plt.imread(path)
    else:
        return None


from keras.models import Sequential
from keras.layers import Flatten, Dense, Conv2D, Lambda, Dropout, MaxPooling2D, Cropping2D
from keras.preprocessing.image import ImageDataGenerator

# Model referencing Nvidia's post here: https://developer.nvidia.com/blog/deep-learning-self-driving-cars/, with some tweaks
def getModel():
    model = Sequential()
    model.add(Cropping2D(cropping=((50,20),(0,0)), input_shape=(160, 320, 3)))
    model.add(Lambda(lambda x: x/255.0-0.5))
    stride_size = 1
    # Instead of stride size 2, added max pool layers with size 2, achieving the same dimension reduction, 
    # but extracting bit more info
    model.add(Conv2D(24, 5, strides=(stride_size,stride_size), activation='relu'))
    model.add(MaxPooling2D((2,2)))
    model.add(Conv2D(36, 5, strides=(stride_size,stride_size), activation='relu'))
    model.add(MaxPooling2D((2,2)))
    model.add(Conv2D(48, 5, strides=(stride_size,stride_size), activation='relu'))
    model.add(MaxPooling2D((2,2)))
    model.add(Conv2D(64, 3, activation='relu'))
    model.add(Conv2D(64, 3, activation='relu'))
    model.add(Flatten())
    # Add a dropout layer between each FC layer
    model.add(Dropout(0.5))
    model.add(Dense(100, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(50, activation='relu'))
    model.add(Dropout(0.5))
    model.add(Dense(10, activation='relu'))
    model.add(Dense(1))

    model.compile(loss='mse', optimizer='adam')
    return model

from keras.models import load_model

def train():
    model = getModel()

    # sets of data for training
    data_paths = [
    # provided base training data for general scenario
    "/opt/carnd_p3/data/", 
    # self collected data on turns with relatively bigger steering angle, clip1
    "/opt/data/sharp1/",   
    # self collected data on turns with relatively bigger steering angle, clip2
    "/opt/data/sharp2/",   
    # self collected data on car swerving around the road, with the segments driving towards sides deleted, only keep the part for recovering to the center
    "/opt/data/recovery"]  

    X_train, y_train = extractImgAndMeasure(data_paths)
    model.fit(X_train, y_train, validation_split=0.2, shuffle=True, nb_epoch=10)

    model.save("model.m5")


train()
