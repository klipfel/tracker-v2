# Robust Deep Learning Tracking Robot -prototype v2

Whilst the [prototype v1](https://github.com/klipfel/tracker-v1) was processing the frames on a remote computer, this prototype is processing the frames on the jetson nano. All the nodes are launched on the jetson nano.

First and foremost, the idea was to try to depend less on the network bandwidth, which is crucial when the frames are sent to a remote computer for processing.

The performances are really slow : 1 FPS. Yet, this repository is the base for the GPU acceleration. To start the `tracker` can be replaced by keeping the same inputs and outputs.

This prototype is **not finished** : it is not possible to launch the project via SSH, since the bouding box cannot be initialized with the display on the remote computer. The display does not go through via SSH, but this is one of the next steps of the development after having the gpu acceleration working on the robot. One idea is to send only the first frame to the remote computer for initialization, and then to use a `roscore` on the robot to diplay the frames.
Here the prototype works entirely on the robot. Thus there is no remote control.

NB : You need a keyboard ... on the robot.

BEFORE DOING ANYTHING : everything is already set up on the Jetson Nano. Just go to the launching part in case you use the right SD card.

# Setup on the Robot

For the installation you can mainly refer to the [prototype v1](https://github.com/klipfel/tracker-v1). 
Exceptions:
* no need to configure the remote computer.
* OpenCV.
* The codes to put the package `robot_tracker` are those in this repository.
* launching procedure.

## Opencv installation on the Robot

/!\ Do this right after the installation of the image.

A version >= 3.4.2 is now needed on the robot.
On the jetson nano the library has to be built from source.
* Install the swapfile : refer to the [prototype v1](https://github.com/klipfel/tracker-v1).
* Create a folder to store the files built files of OpenCV:
```bash
mkdir ~/opencv
```
* Copy the bash file `install_opencv4.0.0_Nano.sh` inside.
Execute the commands:
```bash
chmod +x install_opencv4.0.0_Nano.sh
./install_opencv4.0.0_Nano.sh .
```
* After having installed everything else : ROS, ZED SDK ... the old version of opencv needs to be put aside the 4.0.0.(one is installed by default)
```bash
cd /usr/lib/python2.7/dist-packages/
sudo mkdir opencv3 
sudo rm cv2.aarch64-linux-gnu.so opencv3/cv2.aarch64-linux-gnu.so
```

# Launching on the Robot

NB : switch off the motor and servomotor power supplies.

Open a terminal:
```bash
roscore
```
Open a terminal where the file `tracker.py` is located (on the jetson nano : `/home/hallab/ros_workspace/src/robot_tracker/src`).
Execute the command:
```bash
ROS_HOME='pwd'
roslaunch robot_tracker global.launch
```

NB : If you want to launch just the part on the robot like in the prototype v1:
```bash
roslaunch robot_tracker robot.launch
```
