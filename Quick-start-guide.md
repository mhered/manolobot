# Quick Start Guide

## Start manubot

### Charge the battery

Balance charging the LiPo battery using the iMAX B6AC LiPo balance charger:

1.  Plug the charger to mains to power it up
2. Check that the jacks of the charging cable are correctly connected to the charger (black to negative and red to positive). 
3.  Connect the XT60 connector of the battery to the charging cable. 
4.  Insert the JST-XH balance connector of the battery to the designated slot in the balance charger (In our case 3 cells). 
5.  Use arrows to go to the **Balance Charge** setting.
6.  Select **LiPo** battery type. 
7. Confirm settings. For the 5000mAh 11.1V 50C LiPo battery: **Lipo**,  **3.8A** charging current (below 1C) and **11.1V (3S)** configuration. 
8.  Hold the **Start** button to start. Press it again to confirm. 
9. During charging: in the screen with 6 voltages it is normal to see the lower three indications static at 0.00. This is because the charger is capable of charging batteries with up to 6 cells, but our battery actually has only three.
10. The charger alarm will beep when charge is complete

### Switch on the robot

Red switch on the side from **O** to **I**

### Source ROS2 in the robot

SSH into the robot and source ROS2 workspace:

```bash
(PC T1)$ ssh mhered@manubot
mhered@192.168.8.105's password: 
...
(manubot T1)$ cd ~/dev_ws/
(manubot T1)$ source install/setup.bash
```

### Spawn the robot

```bash
(manubot T1)$ ros2 launch manolobot_uno launch_robot.launch.py
```

### Source ROS2 in the PC

Source ROS2 workspace in PC:

```bash
(PC T2)$ cd ~/dev_ws/
(PC T2)$ source install/setup.bash
```

### Launch RVIZ with config file

Note we use predefined config file but it is not necessary

```bash
(PC T2)$ rviz2 -d ~/dev_ws/src/manolobot_uno/config/bot_with_sensors.rviz
```

### Move the robot

#### With keyboard 

Launch keyboard teleop controller in PC:

```bash
(PC T3)$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Note remapping of topic `/cmd_vel` to `/diff_cont/cmd_vel_unstamped`

Now you can move the robot from computer with the keyboard and see it in RVIZ!

#### With a gamepad

More details in [./BOM/gamepad.md](./BOM/gamepad.md)

Connect the gamepad in Ubuntu:

* ensure it is charged (or charge with microUSB)
* to pair press HOME + SHARE (small button labelled 'S' on the left above the cross) until light flashes white (note if you press HOME + 'O' button on the right the white light blinks, not flashes)
* In Bluetooth settings select Wireless Controller to Connect. Light stops flashing and turns blue
* You may want to test it works with `$ evtest`

Launch gamepad controller:

```bash
$ ros2 launch manolobot_uno joystick.launch.py
```

Note: the controls are defined in a parameter file which implements:

- Dead man switches: L button (left shoulder) for normal speed, R button (right shoulder) for turbo
- Control on left stick: vertical axis for forward/backward motion and horizontal axis for rotation.

### Sensors

#### Stream camera

SSH to the robot from another Terminal and launch the camera controller:

```bash
(manubot T2)$ cd ~/dev_ws/
(manubot T2)$ source install/setup.bash
(manubot T2)$ ros2 launch manolobot_uno camera.launch.py
```

In RVIZ Add Camera and select topic `/image_raw/compressed`

It does not show the camera feed in RVIZ as it should... BUT it works with RQT:

```bash
(PC T4)$ cd ~/dev_ws/
(PC T4)$ source install/setup.bash 
(PC T4)$ ros2 run rqt_image_view rqt_image_view
```

It works!

#### Stream Lidar scans

Launch the LIDAR controller:

```bash
(manubot T3)$ cd ~/dev_ws/
(manubot T3)$ source install/setup.bash
(manubot T3)$ ros2 launch ldlidar_stl_ros2 ld06.launch.py 
```

It works!

## Power off manubot

Safe reset and shutdown: the blue button.

* Shortly pressing the Blue button is ignored (considered noise)
* ~1 sec press resets the RPi (not sure if there is a use case for this)
* 4 sec press shuts down safely the RPi

After Lidar powers off (means RPi powered off) switch off mains red button

See implementation in file `safe_shutdown.py` in: [](/home/mhered/manolobot/code/button/) 

## To Do

- [ ] add photos / video of battery charging
- [ ] fix issue with camera in RVIZ
- [ ] make power down more usable (remove short press for reset and provide feedback for shutdown)
- [ ] wheels skid, big time
- [ ] motors have a lot of hysteresis