#!/usr/bin/env python
from logging import raiseExceptions
from numpy.core.fromnumeric import shape
import rospy
import numpy as np
from rospy.timer import sleep 
from sensor_msgs.msg import JointState
from std_msgs.msg import Header , Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse

class GP8_joint_move():
    def __init__(self):
        self.joint_sub = rospy.Subscriber("motoman_gp8/joint_states",JointState,self.JointStatesCallback)
        #self.joint_sub = rospy.Subscriber("/joint_states",JointState,self.JointStatesCallback)
        self.pub = rospy.Publisher('/motoman_gp8/gp8_controller/command', JointTrajectory, queue_size=1)
        #self.pub = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size=1)
        
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
                self.joint_init_pos = rospy.wait_for_message("motoman_gp8/joint_states",JointState,timeout = 5)
                #self.joint_init_pos = rospy.wait_for_message("/joint_states",JointState,timeout = 5)
            except:
                rospy.logwarn("Time out ")
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
        points.append(self.create_point_msg(self.joint_init_pos.position,rospy.Time.now() + rospy.Duration.from_sec(0.0)))
        points.append(self.create_point_msg(self.new_angle,rospy.Time.now() + rospy.Duration.from_sec(0.0)))
        JointTraj = JointTrajectory()
        JointTraj.joint_names = self.names
        haeder              = Header()
        haeder.frame_id     = ''
        haeder.seq          = 0
        JointTraj.header    = haeder
        JointTraj.points    = points
        self.joint_msg      = JointTraj
    
 
    def pub_joint(self):
        #print(self.joint_msg)
        self.pub.publish(self.joint_msg)
      
    
if __name__ == '__main__':
    rospy.init_node('ManipulatorID', anonymous=True)
   
    man = GP8_joint_move()
   
    new_angle = man.joint_init_pos.position
    new_angle = np.array(new_angle)
    delya = np.array([0, 0, 0.1, 0, 0, 0])

    man.new_angle = new_angle + delya
    print(new_angle,"new_angle")
    print(man.new_angle)
 
    man.set_joint_angles()
    
    while not rospy.is_shutdown():
        #rospy.sleep(0.1)
        man.pub_joint()
       
    