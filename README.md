# Robust Deep Learning Tracking Robot -prototype v2

Whilst the prototype v1 was processing the frames on a remote computer, this prototype is processing the frames on the jetson nano. All the nodes are launched on the jetson nano.

The performances are really slow : 1 FPS. Yet, this repository is the base for the GPU acceleration. To start the `tracker` can be replaced.
This prototype is **not finished**.

# Setup

## Opencv installation

# Launching

Open a terminal where the file `tracker.py` is located.
Execute the command:
```bash
roslaunch robot_tracker global.launch
```

If you want to launch just the part on the robot like in the prototype v1:
```bash
roslaunch robot_tracker robot.launch
```
