# gp8_gazebo_moveit_pkg
This pakege is base on motoman_gp8_support pkg

This pakege contain Gp8 simulation and control, moveit pkg to control simulated and real robot.


**To launch Simulation :

roslaunch gp8_simulation gazebo_gp8.launch


Run moveit for simulation:
roslaunch gp_moveit_config gp8_gazebo_moveit_connection.launch


Moveit for Real Robot:
roslaunch gp_moveit_config gp8_real_moveit_connection.launch


Control real Gp8 joint position:
To control gp8 joint position without moveit run :


control_Real_pg8_joint_path_command.py 

To move the robot acording to your path run in the terminal:

rosservise call /PubTrajService
