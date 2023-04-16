Five Bar Robot
====================
<img src="CAD_Model/Renders/Robot.png" alt="drawing" width="600"/>

Air Hockey Playing Robot
---------------------
Air hockey is used as a demonstration for the five bar robot's control.

Installation instructions
---------------------
Using this [guide](https://roboticsbackend.com/using-ros-on-raspberry-pi-best-practices/)
we downloaded Ubuntu Mate for the raspberry pi.
In tandem, we used this [guide](https://ubuntu-mate.org/raspberry-pi/install/)
to install the right version for our case (arm64 Jammy Jellyfish, 22.04.1 LTS.)
These instructions were not followed exactly, and we used a different flasher called balenaEtcher.

You will have to install a few python packages.

SAMPLE FOR WHAT THIS LOOKS LIKE IN THE README

Refer to the KivyMD page for installing the ui components https://github.com/kivymd/KivyMD
You will have to run these pip install commands for the app to work.
For all other installations, install based on the error messages on import.
```bash
pip install https://github.com/kivy/kivy/archive/master.zip
pip install https://github.com/kivymd/KivyMD/archive/master.zip
```

Hardware Requirements
---------------------
This project was built to run on Ubuntu Mate for the raspberry pi. However, many of the python files included in this
repository can run on Windows machines with no issues.

When first connecting to the ODrive, your computer may have difficultly communicating with the device.
This is often because your computer lacks the drivers required to communicate with the device.
These can be installed by using [Zadig](https://zadig.akeo.ie/), viewing all devices via the options menu, and then
installing "WinUSB" to the native interface for use with the odrivetool. Restart the computer if issues persist.
If none of these options work, refer to the [Odrive documentation](https://docs.odriverobotics.com/v/0.5.5/troubleshooting.html#troubleshooting) for trouble shooting.


The Story
---------------------

### The Inspiration
I enjoy doing fun projects, why else would you make something like this? Actually, it was a project I wanted to do for my
honor's thesis for a class I had been a TA for. I thought the rich dynamics and its design would allow for a really fun
project for the students to work on, so I decided to make it happen!

### What it does
A vision system located above the table tells the 5-bar where to position itself
with respect to the table. This is then fed through a coordinate transform before inverse kinematics are used to determine
possible joint angle that would satisfy the desired positions. At this point, a path is planned between
the current location and the desired location, at which point a following algorithm is used so the 5-bar can make its
way to the final position.