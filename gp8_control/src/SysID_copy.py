#!/usr/bin/env python
from logging import raiseExceptions
from pickle import HIGHEST_PROTOCOL
from threading import current_thread
from numpy.core.fromnumeric import shape
import rospy
import math
import numpy as np
from rospy.core import logwarn_throttle
from rospy.impl.tcpros_service import wait_for_service
from rospy.timer import sleep 
from sensor_msgs.msg import JointState
from std_msgs.msg import Header , Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse
import csv

class GP8_joint_move():
    def __init__(self,mode):
        if mode == 'sim':
            self.joint_sub = rospy.Subscriber("motoman_gp8/joint_states",JointState,self.JointStatesCallback)
            self.pub = rospy.Publisher('/motoman_gp8/gp8_controller/command', JointTrajectory, queue_size=1)
        elif mode == 'real':
            self.joint_sub = rospy.Subscriber("/joint_states",JointState,self.JointStatesCallback)
            self.pub = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size=1)
        else:
            print("incorrect mode")
        
        
        self.joint_init_pos = None
        self.counter    = 0
        self.angles     = []
        self.velocity   = []
        self.new_angle  = []
        self.curr_angle = []
        self.rate = rospy.Rate(50)
        #wait for manipulator joint positions
        while self.joint_init_pos is None:
            try:
                if mode == 'sim':
                    self.joint_init_pos = rospy.wait_for_message("motoman_gp8/joint_states",JointState,timeout = 5)
                elif mode == 'real':
                    self.joint_init_pos = rospy.wait_for_message("/joint_states",JointState,timeout = 5)
            except:
                rospy.logwarn("Time out")
                pass 
        
        self.names = self.joint_init_pos.name
       
       

    def JointStatesCallback(self, msg):
        self.names    = msg.name
        self.angles   = msg.position
        self.velocity = msg.velocity
        self.curr_angle = msg.position
    
        

    def create_point_msg(self,angle,time):
        points = JointTrajectoryPoint()
        points.positions       = angle
        points.velocities      = [0,0,0,0,0,0]
        points.accelerations   = []
        points.effort          = []
        points.time_from_start = rospy.Duration(time)
        return points


    def set_joint_angles(self):
        #print(angle)
        points = []
        points.append(self.create_point_msg(self.curr_angle, 0))
        points.append(self.create_point_msg(self.new_angle,  0.1))
        JointTraj = JointTrajectory()
        JointTraj.joint_names = self.names
        haeder              = Header()
        haeder.frame_id     = ''
        haeder.seq          = 0
        JointTraj.header    = haeder
        JointTraj.points    = points
        self.joint_msg      = JointTraj

    
    def create_empty_msg(self):
        points = []
        JointTraj = JointTrajectory()
        JointTraj.joint_names = self.names
        haeder              = Header()
        haeder.frame_id     = ''
        haeder.seq          = 0
        JointTraj.header    = haeder
        JointTraj.points    = points
        self.empty_msg      = JointTraj
        print(JointTraj)

    def pub_joint(self):
        self.pub.publish(self.joint_msg)
     

    def pub_empty_msg(self):
        self.pub.publish(self.empty_msg)

def limiter(input,high,low):
    for i in range(5):
        if input[i] >= high:
            input[i] = high
        elif input[i] <= low:
             input[i] = low 
        
    return input  
        
    
if __name__ == '__main__':
    f = open('/home/roblab4/catkin_ws/src/matan/test.csv', 'w')
    writer = csv.writer(f)
    
    rospy.init_node('ManipulatorID', anonymous=True)
   
    man = GP8_joint_move(mode = 'sim')

    sign = 1
    
    counter = 0
    
    home = np.array(man.joint_init_pos.position) 
    
    man.new_angle = home
    
    man.set_joint_angles()
   
    man.create_empty_msg()

    rate = rospy.Rate(2)
   
    new_angle = home
    
    ####NOT RATE LIMITED
    # while not rospy.is_shutdown():
     
    #     counter += 1
        
    #     if counter  > 5:
    #         counter = 0
            
    #         sign *= -1
            
    #         joint_com_delt = [0, 0, -0.1, 0 ,0 ,0]

    #         man.new_angle = home + joint_com_delt
  
    #         man.set_joint_angles()
            
    #         man.pub_empty_msg()
            
    #         man.pub_joint()

    #         #print(rospy.Time.now().to_sec)

    #     rate.sleep()
    ########
    ####RATE LIMITED
    while not rospy.is_shutdown():
     
        counter += 1
        
        if counter  > 1:
            counter = 0
            
            sign *= -1
            
            joint_com_delt = [0, 0, 0.1 * sign,0 ,0 ,0]
            
            new_angle =  home + joint_com_delt

        delta = new_angle - man.curr_angle

        # new_angle_rate_limit = man.curr_angle + (delta, 0.01, -0.01)

        new_angle_rate_limit = man.curr_angle + delta

        man.new_angle = new_angle_rate_limit
        
        print(man.new_angle)
        
        man.set_joint_angles()
        
        man.pub_empty_msg()
        
        man.pub_joint()

        rospy.sleep(0.2)
        ############
    

        
        
  

        
        
    