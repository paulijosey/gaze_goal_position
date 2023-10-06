# External Control Library for the RoboDog

## Description
This repo implements ROS(2) nodes for external devices meant as control methods for 
the RoboDog. The main focus lies on Smartglasses/VR/AR Glasses.

## Installation
All packages here are ROS(2) packages so installing them means just cloning this repo 
in your ROS(2) workspace and running "colcon build".
I recommend using the pre made ROS devcontainers from 
https://git.ee.ethz.ch/pbl/pbl-templates/ros_devcontainer. Doing so you do not 
have to manually install dependencies.

Here is a list with the ubuntu and python dependencies if you want to build 
locally (or deploy on a system):

```
sudo apt-get update && sudo apt-get install -y \
    ffmpeg \
    git \
    libboost-all-dev \
    build-essential \
    libglib2.0-dev \
    lua5.3 \
    liblua5.3 \
    libjchart2d-java \
    default-jdk \
    cmake \
    ninja-build \
    stow \
    libsm6 \
    libxext6 \
    python3-dev \
    python3-pip \
    python3-colcon-common-extensions \
    python3-colcon-clean 
```

```
pip install opencv-python \
            pupil-labs-realtime-api \
            numpy>=1.25.2 \
            scipy
```

**IMPORTANT!**: for now the "pupil-labs-realtime-api" is broken and IMU data 
streaming doesn't work as intended. To resolve this a fix has be proposed by 
Mr. Pablo Josè (paul.joseph@pbl.ee.ethz.ch). 
(See PR: https://github.com/pupil-labs/realtime-python-api/pull/47). If this has 
not yet been incorporated in the main repository clone Mr. Pablos branch and build 
it manually.

## Usage
### Pupil Labs Glasses
1. Connect the Glasses to an Android device and start the Neon Companion App. 
Figure out at what IP it is streaming. The ROS node should find the device 
automatically but depending on your wifi setup this might not work (thanks 
eduroam ...). If it doesn't work edit the streamer node IP setting by putting it 
in manually and rebuild the package.
1. Start the streamer node by running 
```ros2 run pupil_labs_glasses pupil_labs_stream_node```
1. Observe the published topics 😎

## Support
If you find any issues with this awesome library fix them yourself 😉. 
Or if you are a bit lazy tell Mr. Pablo (paul.joseph@pbl.ee.ethz.ch) and if 
he's inclined to help he might do so. Also open a gitlab issue to have some 
structure here and keep track of bugs.

## Contributing
I would be happy to get more devices running for this project so any contribution 
would be nice ♥️.

## Authors and acknowledgment
For now only the amazing Pablito Josè (paul.joseph@pbl.ee.ethz.ch).

## License
MIT-License
