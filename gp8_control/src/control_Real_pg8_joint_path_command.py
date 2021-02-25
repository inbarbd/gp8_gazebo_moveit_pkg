#!/usr/bin/env python
'''
----------------------------
Author: Inbar Ben David
        Ten-Aviv University
Date: september 2021
----------------------------
'''

import rospy
import numpy as np 
from sensor_msgs.msg import JointState
from std_msgs.msg import Header , Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse
from motoman_msgs.msg import DynamicJointTrajectory, DynamicJointPoint, DynamicJointsGroup

PATH = [[0.00015339808305725455, -0.08064904808998108, -1.021170973777771, 0.07330382615327835, 0.11543206125497818, 0.41520997881889343], \
    [0.20015339808305727, 0.11935095191001893, -0.821170973777771, 0.27330382615327836, 0.3154320612549782, 0.6152099788188934]]

# JOINT_STATE_TOPIC = "/joint_states"

class GP8_JointPathCommandPublisher(object):
    def __init__(self):
        rospy.loginfo("Enter GP8_JointPathCommandPublisher")
        self.controller_commend = "/joint_path_command"
        self.duration = 1.5
        joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.JointStatesCallback)
        # Check if ROs is loaunch!        
        joints_data = None
        while joints_data is None:
            try:
                joints_data = rospy.wait_for_message("/joint_states", JointState, timeout=5)
                # print(joints_data)
            except:
                rospy.logwarn("Time out ")
                pass

    def JointStatesCallback(self, msg):
        # print("222222222222222222")
        self.names = msg.name
        self.current_angles = msg.position
        self.velocity = msg.velocity

    
    # def NextPosTest(self):
    #     NextPos = []
    #     print(self.current_angles)
    #     for joint in self.current_angles:
    #         NextPosVal = joint+0.2
    #         NextPos.append(NextPosVal)
    #         # print(joint)
    #     print(NextPos)
    #     self.NexPos = NextPos
    #     return NextPos
    

    def BuildTrajFromPath(self,path):
        joint_names = self.names
        start_pt = JointTrajectoryPoint()
        start_pt.positions = self.current_angles
        start_pt.velocities = [0]*len(self.current_angles)
        start_pt.time_from_start = rospy.Duration(0.0)

        end_pt = JointTrajectoryPoint()
        end_pt.positions.append(path[1])  # reorder to match start-pos joint ordering
        end_pt.velocities.append(0)
        end_pt.time_from_start = rospy.Duration(self.duration)

        return JointTrajectory(joint_names=joint_names, points=[start_pt, end_pt]) 

    def JointTrajectoryPoint(self,PosList, time):
        NextPoint = JointTrajectoryPoint()
        if time == 0:
            NextPoint.positions = self.current_angles
            NextPoint.velocities = [0]*len(self.current_angles)
            NextPoint.time_from_start = rospy.Duration(0.0)
        else:
            NextPoint.positions = PosList
            NextPoint.velocities = self.CalculateJointVelocety(time)
            NextPoint.time_from_start = rospy.Duration(time)

        return NextPoint

    def JointTrajectoryMsg(self,path):
        JointTraj = JointTrajectory()
        print(self.names)
        JointTraj.joint_names = self.names

        haeder = Header()
        haeder.frame_id = 'base_link'
        haeder.seq = 0
        JointTraj.header = haeder

        points = []
        for t in range(0,len(path)+1):
            if t == 0:
                points.append(self.JointTrajectoryPoint(path[t],t))
            else:
                points.append(self.JointTrajectoryPoint(path[t-1],t))
        JointTraj.points = points
        # print(len(points))
        return JointTraj


    def move_to_joint(self, path):
        self.path = path
        # print(path[1])
        # print(len(path[1]))
        # print(len(self.names))
        self.traj = self.JointTrajectoryMsg(path)

    def CalculateJointVelocety(self,time):
        velocity_vector = []        
        # print("entrerrrrrrr")
        if time == 1 :
            pos = self.current_angles
            for joint in range(0,len( self.path[time-1])):
                joint_velocity = self.path[time-1][joint] - pos[joint]
                joint_velocity = joint_velocity/self.duration
                velocity_vector.append(joint_velocity)
        else:
            for joint in range(0,len(self.path[time-1])):
                joint_velocity = self.path[time-1][joint] - self.path[time-2][joint]
                joint_velocity = joint_velocity/self.duration
                velocity_vector.append(joint_velocity)
        # print("velocety:", velocity_vector)

        return velocity_vector


class PubTrajService(object):
    def __init__(self,traj):
        self.pub = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size=1)
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

def main_traj(traj):
    # rospy.init_node("PubTrajService_Service")
    a = PubTrajService(traj)
    rospy.Service("PubTrajService",Empty, a.Pub)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('move_group_interface', anonymous=True)
    manipulator = GP8_JointPathCommandPublisher()
    manipulator.move_to_joint(PATH)
    traj = manipulator.traj
    print(traj)
    main_traj(traj)
    print ("THE END")