import csv
import cv2
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle
import sklearn
import tensorflow as tf
import pickle
import keras
from sklearn.externals import joblib
from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Dropout, Cropping2D
from keras.layers import Convolution2D
from keras.layers import MaxPooling2D
from keras.optimizers import Adam

batch_size = 32

def generator2(samples, batch_size=batch_size):
    num_samples = len(samples)
    correction=0.4
    while 1: # Loop forever so the generator never terminates
        shuffle(samples)
        for offset in range(0, num_samples, batch_size):
            batch_samples = samples[offset:offset+batch_size]
            images = []
            angles = []
            for batch_sample in batch_samples:
                name = os.path.join(data_folder, "/".join(batch_sample[0].strip().split('/')[-2:]))
                center_image = cv2.imread(name)
                center_angle = float(batch_sample[3])
                images.append(center_image)
                angles.append(center_angle)
                angles.append(-1.*center_angle)
                images.append(cv2.flip(center_image, 1))
                name_left = os.path.join(data_folder, "/".join(batch_sample[1].strip().split('/')[-2:]))
                left_img = cv2.imread(name_left)
                left_angle = center_angle+correction
                images.append(left_img)
                angles.append(left_angle)
                angles.append(-1*left_angle)
                images.append(cv2.flip(left_img, 1))
                name_right = os.path.join(data_folder, "/".join(batch_sample[2].strip().split('/')[-2:]))
                right_img = cv2.imread(name_right)
                right_angle = center_angle-correction
                images.append(right_img)
                angles.append(right_angle)
                angles.append(-1*right_angle)
                images.append(cv2.flip(right_img, 1))
            X_train = np.array(images)
            y_train = np.array(angles)
            yield sklearn.utils.shuffle(X_train, y_train)


data_folder = "P4Data"
samples = []
with open(os.path.join(data_folder, 'driving_log.csv')) as csvfile:
    reader = csv.reader(csvfile)
    for i, line in enumerate(reader):
        samples.append(line)

train_samples, validation_samples = train_test_split(samples, test_size=0.2)

# compile and train the model using the generator function
train_generator = generator2(train_samples, batch_size=batch_size)
validation_generator = generator2(validation_samples, batch_size=batch_size)

drop_prob = 0.25
n_epochs = 20
class mycb(keras.callbacks.Callback):
    def on_epoch_begin(self, epoch, logs=None):
        lr = self.model.optimizer.lr
        decay = self.model.optimizer.decay
        iterations = self.model.optimizer.iterations
        lr_with_decay = lr / (1. + decay * keras.backend.cast(iterations, keras.backend.dtype(decay)))
        print(keras.backend.eval(lr_with_decay))


cb = mycb()

optimizer = keras.optimizers.Adam(lr=1e-3)
model = Sequential()
model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))
model.add(Cropping2D(cropping=((50,20), (0,0))))
model.add(Convolution2D(24,5, strides=(2,2), activation='relu'))
model.add(Convolution2D(36,5, strides=(2,2), activation='relu'))
model.add(Convolution2D(48,5, strides=(2,2), activation='relu'))
model.add(Convolution2D(64,3, activation='relu'))
model.add(Convolution2D(64,3, activation='relu'))
model.add(Flatten())
model.add(Dense(100, activation='relu'))
model.add(Dropout(rate=drop_prob))
model.add(Dense(50, activation='relu'))
model.add(Dropout(rate=drop_prob))
#model.add(keras.layers.LeakyReLU()) #
model.add(Dense(10, activation='relu'))
#model.add(Dropout(rate=drop_prob))
#model.add(keras.layers.LeakyReLU())
model.add(Dense(1))
model.compile(loss='mse', optimizer=optimizer)
train_stps_epoch = 3*2*len(train_samples)//batch_size #approx
val_stps_epoch = 3*2*len(validation_samples)//batch_size #approx
history = model.fit_generator(train_generator,
    validation_data=validation_generator, epochs=n_epochs,
    steps_per_epoch=train_stps_epoch, validation_steps=val_stps_epoch,
    shuffle=True,
    callbacks=[cb])
with open('trainHistoryDict', 'wb') as file_pi:
    pickle.dump(history.history, file_pi)
model.save("model.h5")
