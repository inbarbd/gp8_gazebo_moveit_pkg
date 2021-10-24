#!/usr/bin/env python
'''
----------------------------
Author: Inbar Ben David
        Ten-Aviv University
Date: september 2021
----------------------------
'''

from re import S
import rospy
import numpy as np 
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header , Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse
from motoman_msgs.msg import DynamicJointTrajectory, DynamicJointPoint, DynamicJointsGroup
# from PublishNexMultyPointTopic import *
from industrial_msgs.srv import StopMotion
from industrial_msgs.msg import RobotStatus

JOINT_STATE_TOPIC = "/joint_states"
PUB_JOINT_TOPIC = '/joint_path_command'
# JOINT_STATE_TOPIC = "/motoman_gp8/joint_states"
# PUB_JOINT_TOPIC = '/motoman_gp8/gp8_controller/command'


class GoNextPoint(object):
    def __init__(self):
        # self.NextP = GeneretPoints()
        rospy.loginfo("Enter GoNextPoint")
        self.duration = 0.001
        rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self.JointStatesCallback)
        rospy.Subscriber('/robot_status',RobotStatus, self.RobotStatusSubscriber)
        rospy.Subscriber('/NexPointTopic',Float64MultiArray,self.NextPointSubscriberStopMotion)
        # rospy.Subscriber('/mrm/joint3_position_controller/command',Float64MultiArray,self.NextPointSubscriberStopMotion)
        
        self.pub = rospy.Publisher(PUB_JOINT_TOPIC, JointTrajectory, queue_size=1)
        # self.traj = traj
        # Check if ROs is loaunch!        
        joints_data = None
        while joints_data is None:
            try:
                joints_data = rospy.wait_for_message(JOINT_STATE_TOPIC, JointState, timeout=5)
                # print(joints_data)
            except:
                rospy.logwarn("Time out ")
                pass
        # rate = rospy.Rate(20)
        rospy.spin()

    def RobotStatusSubscriber(self,msg):
        self.ImMotionFlage = msg.in_motion.val
        # print(msg.in_motion.val)
        # print(type(msg.in_motion.val))

    def NextPointSubscriberStopMotion(self,msg): 
        self.NextCommand = msg.data
        print(self.NextCommand,"next command")      
        # Time = time.time()
        # self.StomMovment =rospy.ServiceProxy('stop_motion', StopMotion)
        # self.StomMovment()
        # print(time.time()-Time,"stop duretion")
        # rospy.loginfo("...StartNewMotion...")
        self.Path = [np.array(self.current_angles),np.array(self.NextCommand)]
        self.traj = self.JointTrajectoryMsg(self.Path)
        # rospy.sleep(2)
        # print(self.traj.points[0].positions,'1')
        rospy.wait_for_message(JOINT_STATE_TOPIC, JointState, timeout=5)
        # rospy.sleep(0.2)
        self.traj.points[0].positions = self.current_angles
        # print(self.traj.points[0].positions,'2')
        self.pub.publish(self.traj)
        # rospy.loginfo('Moving the arm to goal position...')
    
    def NextPointSubscriberStopMotioFlag(self,msg): 
        self.NextCommand = msg.data
        print(self.NextCommand,"next command")      
        # self.StomMovment =rospy.ServiceProxy('stop_motion', StopMotion)
        # self.StomMovment()
        if self.ImMotionFlage ==0:
            rospy.loginfo("...StartNewMotion...")
            self.Path = [np.array(self.current_angles),np.array(self.NextCommand)]
            self.traj = self.JointTrajectoryMsg(self.Path)
            # rospy.sleep(2)
            # print(self.traj.points[0].positions,'1')
            rospy.wait_for_message(JOINT_STATE_TOPIC, JointState, timeout=5)
            # rospy.sleep(0.2)
            self.traj.points[0].positions = self.current_angles
            # print(self.traj.points[0].positions,'2')
            self.pub.publish(self.traj)
            rospy.loginfo('Moving the arm to goal position...')

    def JointStatesCallback(self, msg):
        # print("222222222222222222")
        self.names = msg.name
        self.current_angles = msg.position
        self.velocity = msg.velocity
        # print(msg.header.stamp,"msg")

    def JointTrajectoryPoint(self,PosList, time):
        # print(PosList,"PosList\n",time,"time")
        NextPoint = JointTrajectoryPoint()
        if time == 0:
            # print('1')            
            NextPoint.positions = self.current_angles
            NextPoint.velocities = [0]*len(self.current_angles)
            NextPoint.time_from_start = rospy.Duration(0.0)
        else:
            # print('3')
            NextPoint.positions = PosList
            NextPoint.velocities = self.CalculateJointVelocety(time)
            NextPoint.time_from_start = rospy.Duration(self.duration)
        return NextPoint
          
    def CalculateJointVelocety(self,time):
        velocity_vector = []        
        # print(time,"time")
        for joint in range(0,len(self.Path[time])):
            joint_velocity = self.Path[time][joint] - self.Path[time-1][joint]
            joint_velocity = joint_velocity/self.duration
            velocity_vector.append(joint_velocity)


        return velocity_vector

    def JointTrajectoryMsg(self,path):
        JointTraj = JointTrajectory()
        JointTraj.joint_names = self.names
        haeder = Header()
        haeder.frame_id = ''
        haeder.seq = 0
        JointTraj.header = haeder
        points = []
        for t in range(0,len(path)):
            if t == 0:
                points.append(self.JointTrajectoryPoint(path[t],t))
            else:
                points.append(self.JointTrajectoryPoint(path[t],t))
        JointTraj.points = points
        # print(JointTraj)
        return JointTraj

    def EmptyTrage(self):
        JointTraj = JointTrajectory()
        JointTraj.joint_names = []
        haeder = Header()
        haeder.frame_id = ''
        haeder.seq = 0
        JointTraj.header = haeder
        JointTraj.points = []
        return JointTraj    


class PubTrajService(object):
    def __init__(self,traj):
        # self.pub = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size=1)
        self.pub = rospy.Publisher('/motoman_gp8/gp8_controller/command', JointTrajectory, queue_size=1)
        self.traj = traj
        # print(traj)

    def Pub(self,req):
        rate = rospy.Rate(10)
        # print(traj)
        # g = 0
        # print(g,"pub.get_num_connections(OUT)")
        # while not rospy.is_shutdown():
            # if g <= 1:
        self.pub.publish(self.traj)
                # g = self.pub.get_num_connections()
                # print(g,"pub.get_num_connections()")
        rate.sleep()
        rospy.loginfo("TRAJECTORY IS PUBLISH")
        return EmptyResponse()


if __name__ == '__main__':
    rospy.init_node('GoNextPointTopic', anonymous=True)
    manipulator = GoNextPoint()
