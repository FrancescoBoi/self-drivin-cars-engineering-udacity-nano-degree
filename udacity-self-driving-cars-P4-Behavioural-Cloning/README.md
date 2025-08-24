# Behavioral Cloning Project

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

Overview
---
This repository contains the files for the Behavioral Cloning Project of the Udacity Self-Driving-Cars Nanodegree.

In this project, a convolutional neural network model is trained, validated and tested using Keras. The model is trained and validated on data collected from the Udacity Simulator by the user driving a car around a track. Collected data consist of images, and a csv file (see P4Data archive) where each row has the image filenames , the driving inputs (steering, speed, etc. although in this project only the steering is used). The model outputs a steering angle to drive the car of the simulator autonomously on the track at a constant speed.

The project [writeup](https://github.com/FrancescoBoi/udacity-self-driving-cars-P4-Behavioural-Cloning/blob/master/writeup.pdf) contains a thourough description of the trained model.

The project will contains the following files: 
* [model.py](https://github.com/FrancescoBoi/udacity-self-driving-cars-P4-Behavioural-Cloning/blob/master/model.py): the script used to create and train the model;
* [drive.py](https://github.com/FrancescoBoi/udacity-self-driving-cars-P4-Behavioural-Cloning/blob/master/drive.py): the script to drive the car;
* [model.h5](https://github.com/FrancescoBoi/udacity-self-driving-cars-P4-Behavioural-Cloning/blob/master/model.h5): the file containing the data of the trained Keras model;
* [writeup.pdf](https://github.com/FrancescoBoi/udacity-self-driving-cars-P4-Behavioural-Cloning/blob/master/writeup.pdf): the the report of the project;
* [video.mp4](https://github.com/FrancescoBoi/udacity-self-driving-cars-P4-Behavioural-Cloning/blob/master/video.mp4): the video recording of the vehicle driving autonomously around the track for one full lap.

The following additional files have been added:
* [P4Data.tar.gz](https://github.com/FrancescoBoi/udacity-self-driving-cars-P4-Behavioural-Cloning/blob/master/P4Data.tar.gz): the archive containing the collected data
* [P4OtherData.tar.gz](https://github.com/FrancescoBoi/udacity-self-driving-cars-P4-Behavioural-Cloning/blob/master/P4OtherData.tar.gz): another archive with addional collected data;
* [merge_data.py](https://github.com/FrancescoBoi/udacity-self-driving-cars-P4-Behavioural-Cloning/blob/master/merge_data.py): an utility to merge data coming from different collecting sessions.

This README file describes how to output the video in the "Details About Files In This Directory" section.

The Project
---
The goals / steps of this project are the following:
* Launch the simulator, choose "Simulator mode", press "R" to set the directory where to save data and re-press it to start recording (and then repress it to stop). In my case I recorded three laps around the first track.
* Design, train and validate a model that predicts a steering angle from image data by running the `model.py` script.
* Use the model to drive the vehicle autonomously around the first track in the simulator so that it remains on the road for an entire loop around the track. Launch the simlator and choose "Autonomous mode" and launch the Python script `drive.py`



### Dependencies
This project requires:

* [CarND Term1 Starter Kit](https://github.com/udacity/CarND-Term1-Starter-Kit)

The lab enviroment can be created with CarND Term1 Starter Kit. Click [here](https://github.com/udacity/CarND-Term1-Starter-Kit/blob/master/README.md) for the details.

The following resources can be found in this github repository:
* drive.py
* video.py

The simulator can be downloaded from the classroom. In the classroom, we have also provided sample data that you can optionally use to help train your model.

#### Set TF1 Environment
The project was developed Tensorflow1. The following steps allows to create a TF1 environment.
* Create a conda virtual environment with the following command: `conda create --no-default-packages -n yourEnvName`;
* Activate the virtual environment with `conda activate yourenvname`;
* Run one by one the following commands:
```
conda install -c conda-forge asn1crypto=1.4.0
conda install -c conda-forge blas=1.0
conda install -c conda-forge bzip2=1.0.8
conda install -c conda-forge cacertificates=2020.12.8
conda install -c conda-forge certifi=2020.6.20
conda install -c conda-forge cffi=1.11.5
conda install -c conda-forge click=7.1.2
conda install -c conda-forge cloudpickle=1.6.0
conda install -c conda-forge cryptography=2.3.1
conda install -c conda-forge cycler=0.10.0
conda install -c conda-forge dask-core=2.6.0
conda install -c conda-forge decorator=4.4.2
conda install -c conda-forge eventlet=0.23.0
conda install -c conda-forge ffmpeg=4.0
conda install -c conda-forge flask=1.1.2
conda install -c conda-forge flask-socketio=3.0.1
conda install -c conda-forge freetype=2.8
conda install -c conda-forge greenlet=0.4.15
conda install -c conda-forge h5py=2.8.0
conda install -c conda-forge hdf5=1.10.2
conda install -c conda-forge idna=2.10
conda install -c conda-forge imageio=2.1.2
conda install -c conda-forge intel-openmp=2019.4
conda install -c conda-forge itsdangerous=1.1.0
conda install -c conda-forge jinja2=2.11.2
conda install -c conda-forge jpeg=9b
conda install -c conda-forge kiwisolver=1.0.1
conda install -c conda-forge libcxx=10.0.0
conda install -c conda-forge libffi=3.2.1
conda install -c conda-forge libiconv=1.16
conda install -c conda-forge libopus=1.3.1
conda install -c conda-forge libpng=1.6.37
conda install -c conda-forge libtiff=4.0.9
conda install -c conda-forge libvpx=1.7.0
conda install -c conda-forge markupsafe=1.0
conda install -c conda-forge matplotlib=2.2.2
conda install -c conda-forge mkl=2018.0.3
conda install -c conda-forge ncurses=5.9
conda install -c conda-forge networkx=2.4
conda install -c conda-forge olefile=0.46
conda install -c conda-forge openssl=1.0.2u
conda install -c conda-forge pandas=0.22.0
conda install -c conda-forge patsy=0.5.1
conda install -c conda-forge pip=20.3.3
conda install -c conda-forge pycparser=2.20
conda install -c conda-forge pyopenssl=18.0.0
conda install -c conda-forge pyparsing=2.4.7
conda install -c conda-forge python=3.5.2
conda install -c conda-forge python-dateutil=2.8.1
conda install -c conda-forge python-engineio=3.0.0
conda install -c conda-forge python-socketio=3.0.0
conda install -c conda-forge pytz=2020.4
conda install -c conda-forge pywavelets=1.0.0
conda install -c conda-forge readline=6.2
conda install -c conda-forge scikit-image=0.14.0
conda install -c conda-forge scikit-learn=0.20.0
conda install -c conda-forge seaborn=0.9.0
conda install -c conda-forge setuptools=40.4.3
conda install -c conda-forge six=1.15.0
conda install -c conda-forge sqlite=3.13.0
conda install -c conda-forge statsmodels=0.9.0
conda install -c conda-forge tk=8.5.19
conda install -c conda-forge toolz=0.11.1
conda install -c conda-forge tornado=5.1.1
conda install -c conda-forge werkzeug=1.0.1
conda install -c conda-forge wheel=0.36.2
conda install -c conda-forge xz=5.2.5
conda install -c conda-forge zlib=1.2.11
pip install bleach==1.5.0
pip install chardet==3.0.4
pip install configobj==5.0.6
pip install future==0.18.2
pip install html5lib==0.9999999
pip install imageio-ffmpeg==0.4.2
pip install importlib-metadata==2.1.1
pip install keras==2.2.4
pip install markdown==3.2.2
pip install moviepy==1.0.3
pip install numpy==1.18.5
pip install opencv-python==4.4.0.42
pip install pillow==7.2.0
pip install proglog==0.1.9
pip install protobuf==3.14.0
pip install pyasn1==0.4.8
pip install pyasn1-modules==0.2.8
pip install python-gnupg==0.4.6
pip install python-ldap==3.3.1
pip install pyyaml==5.3.1
pip install requests==2.25.0
pip install scipy==1.4.1
pip install systematic==4.8.7
pip install tensorflow==1.3.0
pip install tensorflow-tensorboard==0.1.8
pip install tqdm==4.54.1
pip install urllib3==1.26.2
pip install zipp==1.2.0
```
These specific versions are required to make the simulator and the `drive.py` communicate. Furthermore, these are compatible with the Udacity GPU virtual machine environment.

## Details About Files In This Directory
### `P4Data.tar.gz`
It is an archive containing the `driving_log.csv` and the `IMG` folder where images get saved. One row of the csv file looks like this:
```
yourSavingFolder/IMG/center_2021_07_28_09_51_40_671.jpg,yourSavingFolder/IMG/left_2021_07_28_09_51_40_671.jpg,yourSavingFolder/IMG/right_2021_07_28_09_51_40_671.jpg,0,1,0,3.763754
```
where the first three columns are paths to the saved images corresponding to the (equivalent of) centre, left and right camera of the car (simulator). The other values are the steering, throttle, break and speed.

### `model.py`
`model.py` trains and saves a keras model in h5 format, using the data specified in the `data_folder` variable (currently set to `P4Data`). The model gets saved with the name `model.h5`. To launch the training:
```sh
python model.py
```
### `drive.py`

Usage of `drive.py` requires one has saved the trained model as an h5 file, i.e. `model.h5`. See the [Keras documentation](https://keras.io/getting-started/faq/#how-can-i-save-a-keras-model) for how to create this file using the following command:
```sh
model.save(filepath)
```

Once the model has been saved, it can be used with drive.py using this command:

```sh
python drive.py model.h5
```

The above command will load the trained model and use the model to make predictions on individual images in real-time and send the predicted angle back to the server via a websocket connection.

Note: There is known local system's setting issue with replacing "," with "." when using drive.py. When this happens it can make predicted steering values clipped to max/min values. If this occurs, a known fix for this is to add "export LANG=en_US.utf8" to the bashrc file.

#### Saving a video of the autonomous agent

```sh
python drive.py model.h5 run1
```

The fourth argument, `run1`, is the directory in which to save the images seen by the agent. If the directory already exists, it'll be overwritten.

```sh
ls run1

[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_424.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_451.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_477.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_528.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_573.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_618.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_697.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_723.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_749.jpg
[2017-01-09 16:10:23 EST]  12KiB 2017_01_09_21_10_23_817.jpg
...
```

The image file name is a timestamp of when the image was seen. This information is used by `video.py` to create a chronological video of the agent driving.

### `video.py`

```sh
python video.py run1
```

Creates a video based on images found in the `run1` directory. The name of the video will be the name of the directory followed by `'.mp4'`, so, in this case the video will be `run1.mp4`.

Optionally, one can specify the FPS (frames per second) of the video:

```sh
python video.py run1 --fps 48
```

Will run the video at 48 FPS. The default FPS is 60.

#### Possible issue
On MacOSX there might be some issues with moviepy and/or ffmpeg versions of the virtual environment. If so, it is better to install `moviepy` with `pip` in the default version of you OS or create another virtual env. Trying to fix this problem in the project virtual environment might result in corrupting the environment itself.

#### Why create a video

1. It's been noted the simulator might perform differently based on the hardware. So if your model drives succesfully on your machine it might not on another machine (your reviewer). Saving a video is a solid backup in case this happens.
2. You could slightly alter the code in `drive.py` and/or `video.py` to create a video of what your model sees after the image is processed (may be helpful for debugging).

### Additional information
- Images are loaded in BGR colorspace using cv2 while drive.py load images in RGB to predict the steering angles.
- If the laptop is slowed down, the simulaor and the `drive.py` do not work well. It is better to reboot the computer or turn it off and wait for a while if it is getting overheated.


