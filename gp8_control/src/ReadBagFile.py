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
from rospy.topics import Publisher 
from sensor_msgs.msg import JointState
from std_msgs.msg import Header , Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse ,Trigger , TriggerResponse  
from gp8_control.srv import float_srv ,float_srvResponse
from motoman_msgs.msg import DynamicJointTrajectory, DynamicJointPoint, DynamicJointsGroup


JOINT_STATE_TOPIC = "/joint_states"
# JOINT_STATE_TOPIC = "/motoman_gp8/joint_states"

class GeneretPoints(object):
    def __init__(self):
        joint_states_sub = rospy.Subscriber(JOINT_STATE_TOPIC, JointState, self.JointStatesCallback)
        # self.PublishPointTopic = rospy.Publisher('NexPointTopic',Float64MultiArray,queue_size=1)
        # Check if ROs is loaunch!        
        joints_data = None
        while joints_data is None:
            try:
                joints_data = rospy.wait_for_message(JOINT_STATE_TOPIC, JointState, timeout=5)
                # print(joints_data)
            except:
                rospy.logwarn("Time out ")
                pass
        self.PublishPointTopic = rospy.Publisher('NexPointTopic',Float64MultiArray,queue_size=1)

    def JointStatesCallback(self, msg):
        # print("222222222222222222")
        self.names = msg.name
        self.current_angles = msg.position
        self.velocity = msg.velocity

    def NextPoint(self,Val):
        self.PointToPublish = Float64MultiArray()
        CurrentP = self.current_angles
        CurrentP =np.array(CurrentP)
        # print(CurrentP)
        deltaAngele = Val
        joint_com_delt = [0, 0, deltaAngele ,0 ,0 ,0]
        new_angle =  CurrentP + joint_com_delt
        # print(new_angle)
        self.nextangle = new_angle
        self.PointToPublish.data = self.nextangle

    
    def Pub(self,req):
        rate = rospy.Rate(3)
        TestPointNumber = 5
        changeDirection = -1
        JointValue = 0.01
        for point in range(TestPointNumber):
            print(point,"point num")
            # print(changeDirection,"changeDirection")
            print(JointValue*changeDirection,"Send angle" )
            self.NextPoint(JointValue*changeDirection)
            changeDirection = -1 * changeDirection  
            # print(changeDirection,"changeDirection")
            self.PublishPointTopic.publish(self.PointToPublish)
            rospy.loginfo("Finish Movment "+str(TestPointNumber)+" Point Num")
            rate.sleep()
        return EmptyResponse()


def main_traj():
    # rospy.init_node("GeneretPoints_Service")
    a = GeneretPoints()
    rospy.Service("ReadBag",Empty, a.Pub)
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('bag_read_node', anonymous=True)
    # GeneretPoints()
    main_traj()