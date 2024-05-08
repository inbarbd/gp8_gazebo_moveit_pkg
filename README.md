# gp8_gazebo_moveit_pkg
This pakege is base on [motoman_ps](https://github.com/MaxorPaxor/motoman_ps])</br> 
In this pkg there is the STL files for the robot_description and the motoman_driver to communicate with the real robot in high and low frequency.</br>

This pakege contain Gp8 simulation and control, moveit pkg to control simulated and real robot.


**Launch Simulation:** 
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

roslaunch gp8_simulation gazebo_gp8.launch </br>
<table>
  <tr>
    <td align="center">
    <!-- <caption>Gazebo Simulation</caption> -->
      <img align=center width=250 src="/video/gazebo_sim_demo.png" />
      <br/>
    </td>
  </tr>
</table>

**Run moveit for simulation:**
 roslaunch gp8_control gp8_robot_and_sim_moveit.launch gazebo_sim=:True

<table>
  <tr>
    <td align="center">
    <!-- <caption>Gazebo Simulation</caption> -->
      <img align=center width=250 src="/video/gazebo_moveit_demo.png" />
      <br/>
    </td>
  </tr>
</table>



**Moveit for Real Robot:**

roslaunch gp8_control gp8_robot_and_sim_moveit.launch gazebo_sim=:False </bt>

exampels for joint_position control and velocity control is at control_gp8_high_rate_control.py


right- position_control exmple </br>
left- velocity control example </br>

<table>
  <tr>
    <td align="center">
    <!-- <caption>Gazebo Simulation</caption> -->
      <img align=center width=250 src="/video/position_control/position_control_gif.gif" />
      <br/>
    </td>
  </tr>
  <tr>
    <td align="center">
    <!-- <caption>Gazebo Simulation</caption> -->
      <img align=center width=250 src="/video/velocity_control/velocity_control_gif.gif" />
      <br/>
    </td>
  </tr>
</table>



