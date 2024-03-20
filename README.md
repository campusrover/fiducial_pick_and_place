# Fiducial Pick and Place

This project has a robotic arm (the Interbotix PX-100) pick up a cargo,
and place it in a drop-off zone. Both the cargo and the drop-off zone
are marked and detected with fiducials.

---
## Table of Contents
- [Video](#video)
- [What](#what)
    - [Hardware](#hardware)
    - [Software](#software)
- [Why](#why)
- [How](#how)
    - [Overview](#overview)
    - [The TF Tree](#the-tf-tree)
    - [The Vision Solution](#the-vision-solution)
    - [Arm Control](#arm-control)
- [Limitations](#limitations)
---

## Video

[Watch the project in action!](https://drive.google.com/file/d/1LnUodQ4iVPImvU8My7JRYx4RfPqLF8GL/view?usp=drivesdk)

## What

The main features of the project are: 

1. the dynamic detection of the camera's location, obtained by the
   transform from a fixed fiducial to the camera; and
2. the ability of the arm to pickup and place a cargo at varying
   locations, so long as both the cargo's and drop-off zone's fiducials
   are:
    1. detected by the camera, and have their x-axes facing away from
       the arm;
    2. within the arm's reach radius; and
    3. within a `[90, -90]` degree range, where degree `0` is the
       direction the arm faces in its sleeping position.

Practically speaking, the first feature allows us to adjust the height
and angle of the camera freely, since its relative location to the
other objects (such as the arm and the cargo fiducial) are not
hard-coded.

As for the second feature, see the [Limitations](#limitations) section
on how its first and third constraints may be overcome.

### Hardware

Besides a desktop computer, the main hardware used was:

1. the PX-100 robotic arm;
2. a usb camera;
3. three fiducial markers;
4. a camera stand; and
3. a 3D-printed cube.

The hardware setup is pictured below:

<p align="center">
    <kbd>
        <img src="./images/hardware_setup.jpg" width="576" height="432" />
    </kbd>
</p>

### Software

The source code was written to run on the Noetic-Ninjemys distribution
of ROS, and thus depends on Ubuntu 20.04. The main ROS packages used
were:

1. `tf2` for managing coordinate frames and transforms;
2. `aruco_detect` for fiducial detection; and
3. `interbotix_xs_modules` for arm manipulation.

Besides the above, the `tf` package was used to convert quaternion
rotations into Euler angles, and the `camera_calibration` package was
used to calibrate the usb camera for fiducial detection. Finally, the
project relied on `numpy` to calculate the distance between the origins
of two coordinate frames.

## Why

This project was developed in the Brandeis Robotics Lab to help
students of COSI 119A: Autonomous Robotics learn how to use the PX-100
Arm. 

## How

### Overview

The source code, found under the `src` directory, consists of two
groups:

1. tf2 broadcasters: the files whose names terminate in
   `_broadcaster.py` make up this group. Each publishes a coordinate
   frame and attaches it to the tf tree which, roughly speaking, helps
   the program determine the relative position of objects (the arm's
   gripper, the fiducials, the cargo, etc.)
2. arm controllers: the files whose names end in `_controller.py`
   are part of this group. Here, the cargo pickup and place logic is
   implemented entirely in the `arm_controller.py` file.

### The TF Tree 

This is what the tf tree looks like:

<p align="center">
    <kbd>
        <img src="./images/tf_tree.png" />
    </kbd>
</p>

The root of the tree is the `world` frame. The subtree that has the
left child of `world` (`px100/base_link`) as its root consists of
frames that help us locate various parts of the PX-100 robotic arm.

On the other hand, the subtree that has the right child of `world`
(`fixed_marker`) as its root are made of frames that constitute our
"vision solution". 

### The Vision Solution

The `fixed_marker` frame is a static frame that is published to be 4.35
inches to the left of the `world` frame, when we view the arm from
above. The two frames can be seen in the RViz screenshot below:

<p align="center">
    <kbd>
        <img src="./images/fixed_marker_2.png" />
    </kbd>
</p>

Overlaid with the model of the robot, the picture looks like this:

<p align="center">
    <kbd>
        <img src="./images/fixed_marker_1.png" />
    </kbd>
</p>

 



The taped fiducial seen in the hardware setup above was placed
such that its coordinate frame overlaps exactly with the `fixed_marker`
frame.

Now, the dimensions of every fiducial is known, and our camera
intrinsics have been calibrated via the `camera_calibration` package.
Therefore, the `aruco_detect` package can use this information  



Brief comment on `test_frame.py` and `test_arm_controller.py`


Show ros graph with different nodes.
Explain how the math works, and why the simple approach of using
transforms doesn't work (4dof). Review dof concept.

### Arm Control

## Limitations


