# LD06 Lidar

Weight: 91g



cfr: https://articulatedrobotics.xyz/mobile-robot-8-lidar/

2D Lidar manufacturer provides a driver that publishes standard ROS messages of type  `sensor_msgs/LaserScan` with the array of measurements and parameters, including the TF where the Lidar is attached so location of measured points in space can be determined.

Similarly 3D Lidar and depth cameras use `sensor_msgs/PointCloud2`

## References

* LD06 small lidar - How to use the Lidar LD06 ROS2: https://www.youtube.com/watch?v=OJWAsV6-0GE

* LDRobot official page: https://www.ldrobot.com/product/en/98

* The download section https://www.ldrobot.com/download/en/98 has Datasheet, Development Manual and SDK (Linux, ROS, ROS2)

* My repo [Minipupper_fun](https://github.com/mhered/minipupper_fun) has information e.g. PC setup [here](https://github.com/mhered/minipupper_fun/blob/main/MiniPupper_SLAM_Navigation.md) and links [here](https://github.com/mhered/minipupper_fun/blob/main/LD06_Links.md)

## Installing LD06 on RPi

1. Clone and symlink the repo:

```bash
(RPi):$ mkdir ~/git && cd ~/git
(RPi):$ git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
(RPi):$ ln -s ~/git/ldlidar_stl_ros2/ ~/dev_ws/src/
```

2. Plug the LD06 and find out the device id (`by-id` because the path `/dev/ttyUSB*` may change):

```bash
(RPi):$ ls /dev/serial/by-id
usb-1a86_USB2.0-Ser_-if00-port0
usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

Where `usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0` is the LD06 ( and`usb-1a86_USB2.0-Ser_-if00-port0` is the arduino)

3. Set device permissions:

```bash
(RPi):$ sudo chmod 777 /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
```

4. Edit `ldlidar_stl_ros2/launch/ld06.launch.py` to update `port_name` and build the package and source:

```bash 
(RPi):$ colcon build
(RPi):$ cd ~/dev_ws
(RPi):$ source install/setup.bash
```

Execute with:

```bash
(RPi):$ ros2 launch ldlidar_stl_ros2 ld06.launch.py
```

View it from PC with:

```bash
(PC):$ rviz2
```

## To do

- [ ] copy lidar launch file to manolobot_uno package
- [ ] make config file for rviz for camera and lidar
