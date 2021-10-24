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
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import sys

from industrial_msgs.srv import StopMotion

CONTROL_GROUP = 'gp8'

class GP8_Moveit_Control(object):
    def __init__(self, control_group):
        super(GP8_Moveit_Control, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        group_name = control_group
        self.move_group = moveit_commander.MoveGroupCommander(group_name)
        self.move_group.set_planner_id("PRMstar")
        self.move_group.set_planning_time(8)
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
        self.planning_frame = self.move_group.get_planning_frame()
        self.group_names = self.robot.get_group_names()
        self.robot.get_current_state()
        
        self.tolerance = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]

    def StopMovment(self):
        print("STOP")
        self.move_group.stop()

    def get_current_position_EE(self):
        return self.move_group.get_current_pose().pose.position
    def get_current_orientain_EE(self):
        a = self.move_group.get_current_pose().pose.orientation  # return orientation in quaternions
        # orien = (np.asarray(tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])) - 2 * np.pi) % (2 * np.pi)
        orien = (np.asarray(tf.transformations.euler_from_quaternion([a.x, a.y, a.z, a.w])))
        return orien


    def set_goal_pose(self, pose, quaternion):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]
        pose_goal.position.x = pose[0]
        pose_goal.position.y = pose[1]
        pose_goal.position.z = pose[2]
        self.move_group.set_pose_target(pose_goal)


    def go_to_pose_goal(self, pose, orientaion):
        """send position and orientaion of the desired point
        pose - x,y,z poistion - in world frame
        orientaion - roll, pitch, yaw position - in world frame
        return true if the movement succeeded and reach at the desired accuracy
        """
        quaternion = tf.transformations.quaternion_from_euler(orientaion[0], orientaion[1], orientaion[2])
        # Planning to a Pose Goal
        self.set_goal_pose(pose, quaternion)

        # we call the planner to compute the plan and execute it.
        plan = self.move_group.go(wait=True)  # return true if succeed false if not
        print(plan, "plan")
        if not plan:
            plan = self.move_group.go(wait=True)  # return true if succeed false if not
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        orientaion_goal = (np.asarray(orientaion)-2 * np.pi) % (2 * np.pi)  # orientaion of the goal
        goal = [pose[0], pose[1], pose[2], orientaion_goal[0], orientaion_goal[1], orientaion_goal[2]]

        orientaion_curr = self.get_current_orientain_EE()  # current orientaion (Rx, Ry, Rz)
        pos_curr = self.get_current_position_EE()  # current position (x,y,z)
        current_loc = [pos_curr.x, pos_curr.y, pos_curr.z, orientaion_curr[0], orientaion_curr[1], orientaion_curr[2]]

        accuracy = self.all_close(goal, current_loc, self.tolerance)
        # diff = [abs(current[j] - goal[j]) for j in range(len(current))]
        # print accuracy, plan, diff
        return accuracy and plan


    def go_to_joint_state(self, goal_joint):
        # Planning to a Joint Goal
        # Todo - fix goal and current joints positions
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = goal_joint[0]
        joint_goal[1] = goal_joint[1]
        joint_goal[2] = goal_joint[2]
        joint_goal[3] = goal_joint[3]
        joint_goal[4] = goal_joint[4]
        joint_goal[5] = goal_joint[5]
        # joint_goal[6] = goal_joint[6]

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()

        current_joints = self.move_group.get_current_joint_values()
        return self.all_close(joint_goal, current_joints, self.tolerance)


    def execute_plan(self, plan):
        # the plan that has already been computed:
        self.move_group.execute(plan, wait=True)

    def all_close(self, goal, actual, tolerance):
        """
        Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
        @param: goal       A list of floats
        @param: actual     A list of floats
        @param: tolerance  A list of floats
        @returns: bool
        """
        for index in range(len(goal)):

            if abs(actual[index] - goal[index]) > tolerance[index]:
                if index > 2:  # for angles
                    if abs(actual[index] - goal[index]) < 2*pi - tolerance[index]:  # 2 pi with tolerance
                        return False
                else:
                    return False
        return True


    def add_obstacles(self, ob_name, pose , size ,timeout=4):
            floor = {'name': ob_name, 'pose': pose, 'size': size}
            # Adding Objects to the Planning Scene
            box_pose = geometry_msgs.msg.PoseStamped()
            box_pose.header.frame_id = self.robot.get_planning_frame()
            box_pose.pose.orientation.w = 1
            box_pose.pose.position.x = floor['pose'][0]
            box_pose.pose.position.y = floor['pose'][1]
            box_pose.pose.position.z = floor['pose'][2]
            self.box_name = floor['name']
            self.scene.add_box(self.box_name, box_pose, size=floor['size'])
            self.scene.attach_box('base_link', self.box_name)
            return self.wait_for_state_update(box_is_known=True, timeout=timeout)


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        # Ensuring Collision Updates Are Receieved
        # If the Python node dies before publishing a collision object update message, the message
        # could get lost and the box will not appear. To ensure that the updates are made, we wait until we see the
        # changes reflected in the ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        start = time.time()  # rospy.get_time()
        seconds = time.time()  # rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self.scene.get_attached_objects([self.box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene. -Note that attaching the box will remove it from known_objects
            is_known = self.box_name in self.scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            time.sleep(0.1)
            seconds = time.time()  # rospy.get_time()
        # If we exited the while loop without returning then we timed out
        return False

class StopMovmentSrv(object):
    def __init__(self):
        # self.a = GP8_Moveit_Control(CONTROL_GROUP)
        self.pub = rospy.Publisher('/joint_path_command', JointTrajectory, queue_size=1)
        # self.pub = rospy.Publisher('/motoman_gp8/gp8_controller/command', JointTrajectory, queue_size=1)
        # print(traj)

    def EmptyTrage(self):
        JointTraj = JointTrajectory()
        JointTraj.joint_names = []
        haeder = Header()
        haeder.frame_id = ''
        haeder.seq = 0
        JointTraj.header = haeder
        JointTraj.points = []
        return JointTraj    


    def Pub(self,req):
        rate = rospy.Rate(10)
        Etraj = self.EmptyTrage()
        # self.a.StopMovment()
        self.pub.publish(Etraj)     
        rate.sleep()
        rospy.loginfo("Stop Movment")
        return EmptyResponse()

class StopMovmentAction(object):
    def __init__(self):
        rospy.loginfo('...connected to system.')


    def Pub(self,req):
        # rate = rospy.Rate(10)
        rospy.loginfo('Stop Recuest Recived')
        self.StomMovment =rospy.ServiceProxy('stop_motion', StopMotion)
        self.StomMovment()
        rospy.loginfo("Stop Movment")
        return EmptyResponse()

def main_traj():
    
    rospy.init_node("StopMovmentSrv_Service")
    a = StopMovmentAction()
    rospy.Service("StopMovmentSrv",Empty, a.Pub)
    rospy.spin()

if __name__ == '__main__':
    # rospy.init_node('move_group_interface', anonymous=True)
    main_traj()
    print ("THE END")