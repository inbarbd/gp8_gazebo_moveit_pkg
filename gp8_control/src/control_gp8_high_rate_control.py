#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import time
import copy
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

"""
 This code demonstrates how to publish and control a robotic arm using a trajectory of points.

 roslaunch motoman_gp8_support robot_interface_streaming_gp8.launch robot_ip:=192.168.255.3 controller:="yrc1000"
 or 
 roslaunch gp8_control gp8_robot_and_sim_moveit.launch gazebo_sim=:False

 rosservice call /robot_enable

 rosrun gp8_control_real_and_sim control_gp8_high_rate_control.py
"""


JOINT_CONTROLLER = '/joint_command' #high rate control REAL_ROBOT
# JOINT_CONTROLLER = '/joint_path_command' #low rate control REAL_ROBOT
JOINT_STATE_TOPIC = '/joint_states' #REAL_ROBOT

# JOINT_CONTROLLER = '/motoman_gp8/gp8_controller/command' #Gazebo control
# JOINT_STATE_TOPIC = '/motoman_gp8/joint_states' #Gazebo control
class gp8_real_time_position_control:
    def __init__(self, rate=1):
        self.rate = rate
        self.duration = 1
        self.step = 1
        self.time_from_start = 0
        self.point = None
        self.old_point = None

        joint_states_sub = rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self.joint_states_callback)
        self.joint_names = None
        self.joint_angles = None
        self.joint_velocity = None

        joints_data = None
        while joints_data is None:
            try:
                joints_data = rospy.wait_for_message(JOINT_STATE_TOPIC, JointState, timeout=5)
            except:
                rospy.logwarn("Time out ")
                pass

        self.publish_topic = rospy.Publisher(JOINT_CONTROLLER , JointTrajectory, queue_size=1)
        self.rate_ros = rospy.Rate(self.rate)

        while self.publish_topic.get_num_connections() == 0:
            self.rate_ros.sleep()
        print("Established Connection")

    def joint_states_callback(self, msg):
        self.joint_names = msg.name
        self.joint_angles = msg.position
        self.joint_velocity = msg.velocity

    def joint_trajectory_msg(self,path):
        self.path = path
        joint_traj = JointTrajectory()

        joint_traj.joint_names = self.joint_names
        joint_traj.header.frame_id = ''
        joint_traj.header.seq = 0
        
        points = []
        for t in range(0,len(self.path)+1):
            if t == 0:
                points.append(self.joint_trajectory_point([],t))
            else:
                points.append(self.joint_trajectory_point(self.path[t-1],t))
        joint_traj.points = points
        # print(len(points))
        return joint_traj

    def joint_trajectory_point(self,PosList, dt):
        # print(len(self.path))
        NextPoint = JointTrajectoryPoint()
        if dt == 0:
            NextPoint.positions = self.joint_angles
            NextPoint.velocities = [0]*len(self.joint_angles)
            NextPoint.time_from_start = rospy.Duration(0.0)
        else:
            NextPoint.positions = PosList
            NextPoint.velocities = self.calculate_joint_velocity(dt)
            NextPoint.time_from_start = rospy.Duration(dt)

        return NextPoint

    def calculate_joint_velocity(self, time):
        velocity_vector = []   
         
        if time == 1 :
            pos = self.joint_angles
            # print(self.path)
            for joint in range(0,len( self.path[time-1])):
                joint_velocity = self.path[time-1][joint] - pos[joint]
                joint_velocity = joint_velocity/self.duration
                velocity_vector.append(joint_velocity)
        if time > 1:
            for joint in range(0,len(self.path[time-1])):
                joint_velocity = self.path[time-1][joint] - self.path[time-2][joint]
                joint_velocity = joint_velocity/self.duration
                velocity_vector.append(joint_velocity)
        # print("velocety:", velocity_vector)

        return velocity_vector

    def publish(self, trajectory):
        self.publish_topic.publish(trajectory)
      
   
class gp8_real_time_velocity_control(object):
    def __init__(self,rate = 2):
        self.rate = rate
        self.total_time = 1.0  # sec
        self.number_steps = int(self.total_time * self.rate)
        self.duration = 1.0 / self.rate
        self.smooth_factor = 0.0  # 10Hz, 0.5sec, 0.5sf
        self.point = None
        self.old_point = None

        self.max_speed_factor = 1.0  # % of max speed for safety reasons
        self.max_speed = np.array([400, 385, 520, 550, 550, 1000])  # deg/s
        print(self.max_speed,"max_speed")
        joint_states_sub = rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self.joint_states_callback)
        
        # Init attributes
        self.curr_time = 0  # sec
        self.curr_step = 0
        self.joint_names = None
        self.angles = None

        joints_data = None
        while joints_data is None:
            try:
                joints_data = rospy.wait_for_message(JOINT_STATE_TOPIC, JointState, timeout=5)
            except:
                rospy.logwarn("Time out ")
                pass

        self.publish_topic = rospy.Publisher(JOINT_CONTROLLER , JointTrajectory, queue_size=1)
        self.rate_ros = rospy.Rate(self.rate)

        while self.publish_topic.get_num_connections() == 0:
            self.rate_ros.sleep()
        print("Established Connection")

    def joint_states_callback(self, msg):
        self.joint_names = msg.name
        self.joint_angles = msg.position
        self.joint_velocity = msg.velocity

    def reset_arm(self):
        """
        Commands robotic arm joints velocities
        """
        self.curr_time = 0  # sec
        self.curr_step = 0

        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()

        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = ''
        trajectory.header.seq = 0

        trajectory.joint_names.append("joint_1_s")
        trajectory.joint_names.append("joint_2_l")
        trajectory.joint_names.append("joint_3_u")
        trajectory.joint_names.append("joint_4_r")
        trajectory.joint_names.append("joint_5_b")
        trajectory.joint_names.append("joint_6_t")

        point.positions.append(self.joint_angles[0])
        point.positions.append(self.joint_angles[1])
        point.positions.append(self.joint_angles[2])
        point.positions.append(self.joint_angles[3])
        point.positions.append(self.joint_angles[4])
        point.positions.append(self.joint_angles[5])

        point.velocities.append(0)
        point.velocities.append(0)
        point.velocities.append(0)
        point.velocities.append(0)
        point.velocities.append(0)
        point.velocities.append(0)

        point.time_from_start = rospy.Duration(1)
        trajectory.points.append(point)
        self.publish_topic.publish(trajectory)

    def vel_trajectory(self, vel_1=0.0, vel_2=0.0, vel_3=0.0, vel_4=0.0,
                       vel_5=0.0, vel_6=0.0, dt=None):
        """
        Commands robotic arm joints velocities
        """

        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()

        trajectory.header.stamp = rospy.Time.now()
        trajectory.header.frame_id = ''
        trajectory.header.seq = 0

        trajectory.joint_names.append("joint_1_s")
        trajectory.joint_names.append("joint_2_l")
        trajectory.joint_names.append("joint_3_u")
        trajectory.joint_names.append("joint_4_r")
        trajectory.joint_names.append("joint_5_b")
        trajectory.joint_names.append("joint_6_t")

        pos_1 = self.joint_angles[0] + vel_1 * 1.0 / self.rate
        pos_2 = self.joint_angles[1] + vel_2 * 1.0 / self.rate
        pos_3 = self.joint_angles[2] + vel_3 * 1.0 / self.rate
        pos_4 = self.joint_angles[3] + vel_4 * 1.0 / self.rate
        pos_5 = self.joint_angles[4] + vel_5 * 1.0 / self.rate
        pos_6 = self.joint_angles[5] + vel_6 * 1.0 / self.rate

        point.positions.append(pos_1)
        point.positions.append(pos_2)
        point.positions.append(pos_3)
        point.positions.append(pos_4)
        point.positions.append(pos_5)
        point.positions.append(pos_6)

        point.velocities.append(vel_1)
        point.velocities.append(vel_2)
        point.velocities.append(vel_3)
        point.velocities.append(vel_4)
        point.velocities.append(vel_5)
        point.velocities.append(vel_6)

        # point.velocities.append(0)
        # point.velocities.append(0)
        # point.velocities.append(0)
        # point.velocities.append(0)
        # point.velocities.append(0)
        # point.velocities.append(0)

        if dt is None:
            dt = 1.0 / self.rate

        point.time_from_start = rospy.Duration(dt * self.curr_step)
        trajectory.points.append(point)

        return trajectory

    def first_step(self, velocity_vector):
        self.velocity = velocity_vector
        vel_1, vel_2, vel_3, vel_4, vel_5, vel_6 = velocity_vector

        trajectory = self.vel_trajectory(vel_1, vel_2, vel_3, vel_4, vel_5, vel_6)
        self.publish_topic.publish(trajectory)

        time.sleep(5.0 / self.rate)
        self.curr_time += 1.0 / self.rate
        self.curr_step += 1

    def step(self, velocity_vector):
        """
        Performs 1 step for the robotic arm and checks if stop conditions are met
        Returns reward, done flag and termination reason
        """       
        vel_1, vel_2, vel_3, vel_4, vel_5, vel_6 = velocity_vector

        trajectory = self.vel_trajectory(vel_1, vel_2, vel_3, vel_4, vel_5, vel_6)
        self.publish_topic.publish(trajectory)
        # print(trajectory)


        time.sleep(1.0 / self.rate)
        # self.rate.sleep()
        self.curr_time += 1.0 / self.rate
        self.curr_step += 1

        """
        Stop if
        - Time is over
        - Gripper was opened
        - Object touched the ground
        """

        if self.curr_step >= self.number_steps:
            termination_reason = "Time is up: {}".format(self.curr_time)
            done = True

        else:  # Gripper is closed and time is not up
            termination_reason = None
            done = False
        return done, termination_reason
       
def example_position_control_low_rate_publish():
    position_manipulator = gp8_real_time_position_control()
    genetate_path  = []
    index = 1
    for i in range (0,11):
        current_joint_state = position_manipulator.joint_angles
        current_joint_state = list(current_joint_state)
        current_joint_state[0] = current_joint_state[0] + index*0.2
        genetate_path.append(current_joint_state)
        index = -1*index
    trajectory = position_manipulator.joint_trajectory_msg(genetate_path)
    print(trajectory,"trajectory")
    position_manipulator.publish(trajectory)

def example_velocity_control_high_rate_publish():
    velocity_manipulator = gp8_real_time_velocity_control()
    velocity_manipulator.reset_arm()
    PATH = [[0.5,0,0,0,0,0],
                [-0.5,0,0,0,0,0],
                [0.5,0,0,0,0,0],
                [-0.5,0,0,0,0,0],
                ]

    # Publish current position at first
    trajectory = velocity_manipulator.first_step([0,0,0,0,0,0])
    # velocity_manipulator.publish(trajectory)
    
    for _ in range(1):
        for p in PATH:
            time.sleep(0.003)  # Simulates network pass
            trajectory = velocity_manipulator.step(p)
            # print(trajectory)
            # velocity_manipulator.publish_topic.publish(trajectory)




if __name__ == '__main__':
    rospy.init_node('move_gp8_high_rate_control', anonymous=True)
    # example_position_control_low_rate_publish()
    example_velocity_control_high_rate_publish()

    
    