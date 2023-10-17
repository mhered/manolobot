# Starting manubot

## Charge the battery

Remember during charging: in the screen with 6 voltages it is normal to see the lower 3 at 0.00 the charger is capable to dealing with 6 cell batteries but my battery has only 3 cells (11.1V)

## Switch on the robot

Red switch on the side

## Run ROS in the robot

SSH into the robot and source ROS2 workspace:

```bash
(PC T1)$ ssh mhered@manubot
mhered@192.168.8.105's password: 
...
(manubot T1)$ cd ~/dev_ws/
(manubot T1)$ source install/setup.bash
```

## 

## Spawn the robot

```bash
(manubot T1)$ ros2 launch manolobot_uno launch_robot.launch.py
```

## Run ROS commands in PC

Source ROS2 workspace in PC:

```bash
(PC T2)$ cd ~/dev_ws/
(PC T2)$ source install/setup.bash
```

## Launch RVIZ with config file

Note we use predefined config file but it is not necessary

```bash
(PC T2)$ rviz2 -d ~/dev_ws/src/manolobot_uno/config/bot_with_sensors.rviz
```

## Launch keyboard teleop in PC

```bash
(PC T3)$ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diff_cont/cmd_vel_unstamped
```

Note remapping of topic `/cmd_vel` to `/diff_cont/cmd_vel_unstamped`

Now you can move the robot from computer with the keyboard and see it in RVIZ!

## Launch camera controller

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

## Launch LIDAR controller

```bash
(manubot T2)$ cd ~/dev_ws/
(manubot T2)$ source install/setup.bash
(manubot T2)$ ros2 launch ldlidar_stl_ros2 ld06.launch.py 
```

It works!

# Power off manubot

Safe reset and shutdown: the blue button.

* Shortly pressing the Blue button is ignored (considered noise)
* ~1 sec press resets the RPi (not sure if there is a use case for this)
* 4 sec press shuts down safely the RPi

After Lidar powers off (means RPi powered off) switch off mains red button

See implementation in file `safe_shutdown.py` in: [](/home/mhered/manolobot/code/button/) 