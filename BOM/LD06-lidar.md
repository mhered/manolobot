# LD06 Lidar

LD06 small lidar - How to use the Lidar LD06 ROS2: https://www.youtube.com/watch?v=OJWAsV6-0GE

LDRobot official page: https://www.ldrobot.com/product/en/98

The download section https://www.ldrobot.com/download/en/98 has Datasheet, Development Manual and SDK (Linux, ROS, ROS2)

My repo [Minipupper_fun](https://github.com/mhered/minipupper_fun) has information e.g. PC setup [here](https://github.com/mhered/minipupper_fun/blob/main/MiniPupper_SLAM_Navigation.md) and links [here](https://github.com/mhered/minipupper_fun/blob/main/LD06_Links.md)



2D Lidar manufacturer provides a driver that publishes standard ROS messages of type  `sensor_msgs/LaserScan` with the array of measurements and parameters, including the TF where the Lidar is attached so location of measured points in space can be determined.

Similarly 3D Lidar and depth cameras use `sensor_msgs/PointCloud2`
