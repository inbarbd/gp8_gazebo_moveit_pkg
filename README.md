# gp8_gazebo_moveit_pkg
This pakege is base on motoman_gp8_support pkg

This pakege contain Gp8 simulation and control, moveit pkg to control simulated and real robot.


**Launch Simulation:** 

roslaunch gp8_simulation gazebo_gp8.launch


**Run moveit for simulation:**

roslaunch gp_moveit_config gp8_gazebo_moveit_connection.launch
<img src=" <a href="https://ibb.co/rQ0wmqk"><img src="https://i.ibb.co/ZfXMVvd/Gp8-Simulation.png" alt="Gp8-Simulation" border="0"></a>" width="500">

**Moveit for Real Robot:**

roslaunch gp_moveit_config gp8_real_moveit_connection.launch


**Control real Gp8 joint position:**

Run :


control_Real_pg8_joint_path_command.py 

To move the robot according to your path run in the terminal:

rosservise call /PubTrajService
