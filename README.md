# Fiducial Pick and Place

---
## Table of Contents
- [Video](#video)
- [What](#what)
    - [Hardware](#hardware)
    - [Software](#software)
- [Why](#why)
- [How](#how)
---

## Video

[Watch the project in action!](https://drive.google.com/file/d/1LnUodQ4iVPImvU8My7JRYx4RfPqLF8GL/view?usp=drivesdk)

## What

This project has a robotic arm (the Interbotix PX-100) pick up a cargo,
and place it in a drop-off zone. Both the cargo and the drop-off zone
are marked with fiducials. 

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
3. three fiducial markers.
4. a camera stand; and
3. a 3D-printed cube.

The hardware setup is pictured below:

<p align="center">
    <kbd>
        <img src="./images/hardware_setup.jpg" width="576" height="432" />
    </kbd>
</p>

### Software

The source code runs on ROS-Noetic in the

ROS-Noetic. Ubuntu Version. Main packages used (`aruco_detect`, `tf2`).
Numpy for calculation of distance between frame origins.

## Why

Developed as an example project for future students of Brandeis
Robotics Lab.

## How

Show ros graph with different nodes.
Explain how the math works, and why the simple approach of using
transforms doesn't work (4dof). Review dof concept.

## Limitations


