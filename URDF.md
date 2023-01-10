# Describing the robot in URDF

## Prerequisites

- [x] installed ROS

- [x] watched Intro video [Why do I think you should build this robot?](https://www.youtube.com/watch?v=OWeLUSzxMsw&t=0s)

  - [x] copied template from github: https://github.com/joshnewans/my_bot, created a workspace and built with colcon:
  
  ```bash
  $ cd ~ 
  $ mkdir dev_ws && cd dev_ws
  $ mkdir src && cd src
  $ git clone https://github.com/mhered/manolobot_uno.git
  $ cd ..
  $ colcon build --symlink-install
  ```
  
- [x] watched TF overview video: [ROS Transform System (TF) | Getting Ready to Build Robots with ROS #6](https://www.youtube.com/watch?v=QyvHhY4Y_Y8&t=0s) 

  * Played with static transforms: `$ ros2 run tf2_ros static_transform_publisher x y z yaw pitch roll parent_frame child_frame` with translations and rotations (in radians) processed in the order they appear in the command.

  * to open RVIZ: `$ ros2 run rviz2 rviz2` or simply `$ rviz2`
  * [x] installed xacro and joint state publisher GUI:

    ```bash
    $ sudo apt install ros-foxy-xacro ros-foxy-joint-state-publisher-gui
    ```

  * [x] played with dynamic transforms
      - `robot_state_publisher` node takes URDFs and publishes transforms in `/tf` and `/tf_static` topics. We need to publish `/joint_states` messages e.g. using `joint_state_publisher_gui`
      -  It takes as parameter the complete contents (!) of the URDF file, which is why we add `"$( xacro /path-to-file )"`:

      ```bash
      (Terminal 1):$ ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro ~/example_robot.urdf.xacro )"
      ```

      ```bash
      (Terminal 2):$ ros2 run joint_state_publisher_gui joint_state_publisher_gui
      ```

  * [x] tried debugging with `view_frames`

    * Similar to `rqt_tf_tree` from ROS1, which does not work in ROS2
    * Run `view_frames` with:

    ```bash
    $ ros2 run tf2_tools view_frames.py
    ```
    * Listens to transforms 5 secs then writes a `frames.gv` and `frames.pdf` to the local directory

- [x] watched URDF Overview video [How do we describe a robot? With URDF! | Getting Ready to build Robots with ROS #7](https://www.youtube.com/watch?v=CwdbsvcpOHM)
  * URDF: Unified Robot Description Format, based on XML
  * Describes the robot as a tree of **Links** connected by **Joints** - where each Link may have many childs but has only one parent (tree structure without closed loops)
  * Root tag of the XML is `robot`, its only attribute is `name`
  * We assign a **Link** to every part that moves relative to others or can be physically detached (hence will require calibration). A **Link** is defined by a `name` and also its `visual`, `collision` and `inertial` properties. We can define multiple `visual`, `collision` tags to define complex shapes.
  * **Joints** define the position and orientation of the child with respect to the parent Link. 4 most common Joint types are `revolute` (rotate with limits), `continuous` (rotate freely), `prismatic` (linear slider) and  `fixed`. a **Joint** is defined by `name`,` type`, `parent` and `child`, `origin` (initial position),  and for non fixed joints also`axis`, and `limits` (`min`and `max` are position in rad or m, `velocity` is max in rad/s or m/s and `effort` is max in N or Nm). 
  * Naming convention: `arm_link` is the child link in`arm_joint`
  
  ```xml
  <?xml xmlns:xacro="http://www.ros.org/wiki/xacro" version="1.0"?>
  <robot name="example_urdf">
      <!-- example Link -->
      <link name="base_link">
      	<visual> what we see in RVIZ and gazebo
              <geometry></geometry> overall shape: box, cylinder, sphere or path to 3d mesh 
              <origin></origin> offset to geometry. Origin can be selected freely except for rotating Links that must have origin at pivot point.
              <material></material> color as RGB triplet or name of one defined previously 
          </visual>
          <collision> used for physics collision calculations. Copy & paste of visual or simplified for better performance.
              <geometry></geometry>
              <origin></origin>
          </collision>
          <inertial> for calculations of response to forces
              <mass></mass>
              <origin></origin> center of mass
              <inertia></inertia> rotational inertia matrix. approximated from primitives or calculated with CAD program for meshes
          </inertial>
      </link>
      
      <!-- example Joint -->
      <joint name="arm_joint" type = "revolute">
          <parent link="slider_link" />
          <child link="arm_link" />
          <origin xyz="0.25 0 0.12" rpy="0 0 0" />
          <axis xyz="0 -1 0" />
          <limit lower="0" upper="${pi/2}" velocity="100" effort="100">
      </joint>
  
  </robot>
  ```
  
  * `xacro` XML macro allows to include subfiles in a main file e.g to split into smaller reusable files. To enable it add `xmlns:xacro="http://www.ros.org/wiki/xacro"` to the `robot` tag. The include statement is e.g.: `<xacro:include filename="my_materials.xacro" />`
  
  * That's why we normally process the raw URDF file(s) with xacro and feed it directly to the `robot_state_publisher` 
  
  * Also has properties and mathematical operators e.g.  `<xacro:property name="r" value ="0.5" />` used later to define a cylinder  `<cylinder radius="${r}" length="${4*r}">`
  
  * Also has macros that expand replacing parameters, e.g.:
  
      ```xml
      <xacro:macro name="inertial_box" params ="mass x y z *origin">
          <inertial>
              <xacro:insert_block name="origin" />
              <mass value="${mass}" />
              <inertia ixx="${(1/12) * mass * (y*y+z*z)}" etc />
          </inertial>
      </xacro:macro>
      ```
  
    Can be used as:
  
    ```xml
    <xacro:inertial_box mass="12" x="2" y="3" z="4">
        <origin xyz="0 2 4" rpy="0 0 0" />
    </xacro:inertial_box>
    ```
  
    And expands to:
  
    ```
        <inertial>
            <origin xyz="0 2 4" rpy="0 0 0" />
            <mass value="12" />
            <inertia ixx="25" etc />
        </inertial>
    ```
  
    

## Let's start

Main video: [Creating a rough 3D model of our robot with URDF](https://youtu.be/BcjHyhV0kIs) 
Blog: [Making a Mobile Robot #2 - Concept Design URDF](https://articulatedrobotics.xyz/mobile-robot-2-concept-urdf/) 
Discussion: [Discourse](https://discourse.articulatedrobotics.xyz/t/discussion-concept-design-urdf-making-a-mobile-robot-pt-2/27) 

Notes:

* root link must be called `base_link`
* orientation is:

  - x forward (red)

  - y left (green)

  - z up (blue)

3 things to take into consideration when saving changes in URDF:

1. build workspace with `$ colcon build --symlink-install`. This avoids having to rebuild with every change (except if you add new files)

2. quit (ctrl+c) and relaunch the robot state publisher with. `$ ros2 launch manolobot rsp.launch.py`every time you make a change

3. click **Reset** to refresh RVIZ so it picks up changes. If this does not work tick/untick display items or close/reopen as last resource
