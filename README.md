# Teach-Industrial-Robots
Nowadays a lot of people want to use a NUI (Natural User Interface). Therefore we want to adapt this idea to the area of teaching industrial robots with gestures.

## Required libraries
* GLM (OpenGL mathematics)
  ```console
  sudo apt install libglm-dev
  ```
* Eigen3 (a high-level C++ library of template headers for linear algebra, matrix and vector operations, geometrical transformations, numerical solvers and related algorithms)
  ```console
  sudo apt install libeigen3-dev
  ```
* RapidJSON (a JSON parser and generator for C++)
  ```console
  sudo apt install rapidjson-dev
  ```
* libk4a (Azure Kinect Sensor)
  ```console
  sudo apt install libk4a1.3-dev
  ```
* libk4abt (Azure Kinect Body Tracking)
  ```console
  sudo apt install libk4abt1.0-dev
  ```
* interbotix_ros_arms\
  A modified version of [interbotix_ros_arms](https://github.com/Interbotix/interbotix_ros_arms) hosted on [https://github.com/hrsFire/interbotix_ros_arms](https://github.com/hrsFire/interbotix_ros_arms)

## Build Tool
This project uses catkin-tools.
```console
sudo apt-get install python-catkin-tools
```

## How to launch (Terminal)
1. Use the [quickstart](https://github.com/Interbotix/interbotix_ros_arms#quickstart) from interbotix_ros_arms, but clone from the following repository which contains a patch which allows the node to be build als a library:
   git@github.com:hrsFire/interbotix_ros_arms.git
2. Clone this project in the src directory of a ROS workspace
3. Use catkin_make to compile the packages in the the ROS workspace
4. Source now the ROS workspace again
4. Run the app:
   ```console
   rosrun teach_industrial_robots teach_industrial_robots_node
   ```

## How to launch (VS Code)
1. Clone the interbotix_ros_arms as stated in "How to launch (Terminal)"
2. Use catkin_make to compile the packages in the the ROS workspace for the first time. Afterwards the code can be compiled in VS Code.
3. Source now the ROS workspace again
4. Launch VS Code
5. Use the shortcut "ctrl+shift+B" to build the project and all of its dependencies.
6. Start the project with the shortcut "F5" for debug mode or "ctrl+F5" for launching the app without debugging support.