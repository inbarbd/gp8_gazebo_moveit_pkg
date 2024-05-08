# gp8_gazebo_moveit_pkg
This pakege is base on [motoman_ps](https://github.com/MaxorPaxor/motoman_ps])</br> 
In this pkg there is the STL files for the robot_description and the motoman_driver to communicate with the real robot in high and low frequency.</br>

This pakege contain Gp8 simulation and control, moveit pkg for simulated and real robot control.


**Launch Simulation:** </br>
To launch the robot description with joint command gui run : </br>
roslaunch gp8_simulation rviz_gp8_robot_discription.launch </br>

<table align="center">
  <tr>
    <td align="center">
    <!-- <caption>Gazebo Simulation</caption> -->
      <img align=center width=250 src="/video/rviz_launch_demo.gif" />
      <br/>
    </td>
  </tr>
</table>

To launch gp8 in gazebo simulation run : </br>
roslaunch gp8_simulation gazebo_gp8.launch </br>
<table align="center">
  <tr>
    <td align="center">
    <!-- <caption>Gazebo Simulation</caption> -->
      <img align=center width=250 src="/video/gazebo_sim_demo.png" />
      <br/>
    </td>
  </tr>
</table>

**Run moveit for simulation:** </br>
To launch gazebo simulation with moveit control run: </br>
roslaunch gp8_control gp8_robot_and_sim_moveit.launch gazebo_sim=:True </br>

<table align="center">
  <tr>
    <td align="center">
    <!-- <caption>Gazebo Simulation</caption> -->
      <img align=center width=250 src="/video/gazebo_moveit_demo.gif" />
      <br/>
    </td>
  </tr>
</table>



**Moveit for Real Robot:**
To launch real robot connection with low high rate control and moveit pkg, run: </br>
roslaunch gp8_control gp8_robot_and_sim_moveit.launch gazebo_sim=:False </bt>

Exampels for joint_position control and velocity control is at control_gp8_high_rate_control.py </bt>


right- position_control exmple </br>
left- velocity control example </br>

<table>
  <tr>
    <td align="center">
    <!-- <caption>Gazebo Simulation</caption> -->
      <img align=center width=250 src="/video/position_control/position_control_gif.gif" />
      <br/>
    </td>
    <td align="center">
    <!-- <caption>Gazebo Simulation</caption> -->
      <img align=center width=250 src="/video/velocity_control/velocity_control_gif.gif" />
      <br/>
    </td>
  </tr>
</table>



