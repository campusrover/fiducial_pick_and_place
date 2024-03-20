# Fiducial Pick and Place

This project has a robotic arm (the Interbotix PX-100) pick up a cargo,
and place it in a drop-off zone. Both the cargo and the drop-off zone
are marked by, and detected with, fiducials.

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
        - [The Fiducial Setup](#the-fiducial-setup)
        - [The Problem](#the-problem)
        - [The Solution](#the-solution)
- [Limitations](#limitations)
- [Miscellaneous](#miscellaneous)
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
inches to the right of the `world` frame, when we view the arm from
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

The fiducial taped to the platform, seen in the hardware setup picture
above, was positioned such that its coordinate frame overlaps exactly
with the `fixed_marker` frame.

Now, the dimensions of every fiducial is known, and our camera
intrinsics have been calibrated via the `camera_calibration` package.
So the `aruco_detect` package can use this information to publish
transforms between the taped fiducial and the usb camera.

But `aruco_detect` by itself doesn't "know" where the taped fiducial is
located or where the usb camera is located "in the real world". It only
knows the transforms between the fiducial and camera frames, _whereever
they may be_.
 
This is where we can use the `fixed_marker` frame we mentioned above to
"anchor" the coordinate frames of both the fiducial and the camera.
After all, we know where the `fixed_marker` frame is in the real world:
4.35 inches to the right of the center of the base of the robot. 

So we can take the transform from the taped fiducial to the camera,
provided by `aruco_detect`, and apply its translation and rotation to
the `fixed_marker` frame, to deduce the location of the camera.

This, effectively, is what the `camera_frame_broadcaster.py` file does,
in publishing a frame for the camera as a child of the `fixed_marker`
frame, by using the transform from `fiducial_0` to `usb_cam`. The
result can be seen below. Notice how the position of the `fiducial_0`
frame, given by `aruco_detect`, overlaps with that of the
`fixed_marker` frame.

<p align="center">
    <kbd>
        <img src="./images/usb_cam.png" />
    </kbd>
</p>

And this is how we get the dynamic detection of the camera's location,
mentioned above. Since our `fixed_marker` frame serves as the anchor
for our `usb_cam` frame, we can move our `usb_cam` freely and expect
our system to detect it, so long as the camera keeps `fiducial_0`
within its field of vision.

### Arm Control

#### The Fiducial Setup

Thus, with our vision solution, the `usb_cam` frame can be successfully
registered in the tf tree as a child of the `fixed_marker` frame. And
so long as the camera keeps the fixed fiducial (`fiducial_0`) in view,
any other fiducial that it sees via `aruco_detect` would also have its
frame automatically registered as a node of the tf tree.

This is how we see, in our diagram of the tree above, `fiducial_1` and
`fiducial_2` being children of `usb_cam`, where the former is the
fiducial fixed to the cargo, and the latter is that representing the
drop-off zone.

The picture that emerges in RViz is as follows:

<p align="center">
    <kbd>
        <img src="./images/fiducial_setup.png" />
    </kbd>
</p>

#### The Problem

Given this fiducial setup, we would expect controlling the arm to be as
simple as taking the transform from some component of the arm to, say,
the cargo fiducial, and directing the robot's end effector to match its
frame to that of the cargo fiducial.

Unfortunately, the PX-100 has less than 6 degrees of freedom, and so it
cannot position its end-effector's frame to match the pose of any
arbitrary frame within its workspace. So our arm control algorithm must
be a bit more complex, and function within the hardware constraints of
the robot.

#### The Solution

Fundamentally, then, the PX-100 can be thought of as a linear gripper
on a swivel. In other words, the arm must directly face whatever it
would grab; before it can grab any object, there must be a straight
line between the object and the middling area of the robot's end
effector.

***=====RESUME HERE=====***

1. Illustrate this point with the test frame and the
   `test_arm_controller`. Add more screenshots of RViz.

But the requirement that a user perfectly align the cargo and the
drop-off zone's fiducial's x-axis to an imagined straight line from the
center of the robot to the cargo is not only unrealistic, but also
impractical, even for an educational teaching project.

A solution would be as follows.
1. find the distance from the origin of the `base_link` to the
   fiducial.
2. publish a frame that shares its origin with the fiducial, but has
   the same orientation as the `base_link`'s frame.
3. Use the frame to calculate the "opposite". The distance is the
   "hypotenuse". Then use arcsin to derive the angle of the yaw.


## Limitations

## Miscellaneous

The test frame.
