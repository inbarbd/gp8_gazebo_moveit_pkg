# gp8_gazebo_moveit_pkg
This pakege is base on motoman_gp8_support pkg

This pakege contain Gp8 simulation and control, moveit pkg to control simulated and real robot.


**Launch Simulation:** 

roslaunch gp8_simulation gazebo_gp8.launch


**Run moveit for simulation:**

roslaunch gp_moveit_config gp8_gazebo_moveit_connection.launch
<img src="https://i.ibb.co/ZfXMVvd/Gp8-Simulation.png" alt="Gp8-Simulation" border="0"></a>

**Moveit for Real Robot:**

roslaunch gp_moveit_config gp8_real_moveit_connection.launch

<a href="https://ibb.co/0CcGD1Y"><img src="https://i.ibb.co/487VmC4/Gp8-Moveit.png" alt="Gp8-Moveit" border="0"></a>

**Control real Gp8 joint position:**

Run :


control_Real_pg8_joint_path_command.py 

To move the robot according to your path run in the terminal:

rosservise call /PubTrajService
