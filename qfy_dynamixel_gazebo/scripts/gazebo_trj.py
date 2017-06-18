#!/usr/bin/env python

# @Author:qfyhaha
# @Description:
import roslib

roslib.load_manifest('qfy_dynamixel')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg
import control_msgs.msg
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, \
    FollowJointTrajectoryGoal
from qfy_dynamixel.msg import multi_joint_point



class Joint:
    def __init__(self, motor_name):
        # arm_name should be b_arm or f_arm
        self.name = motor_name
        self.mx_joint_id = []
        self.ax_joint_id = []
        self.ax_joint_data = []
        self.mx_joint_data = []

        self.sub = rospy.Subscriber('joint_goal_point',multi_joint_point,self.callback)

        self.jta = actionlib.SimpleActionClient('/' + self.name + '_controller/follow_joint_trajectory',
                                                FollowJointTrajectoryAction)
        rospy.loginfo('Waiting for joint trajectory action')
        if self.jta.wait_for_server():
            rospy.loginfo('Server Started')
        # rospy.loginfo('Found joint trajectory action!')

    def move_joint(self, angles):
        goal = FollowJointTrajectoryGoal()
        if self.name[0] == 'm':
            goal.trajectory.joint_names = self.mx_joint_id
        else:
            goal.trajectory.joint_names = self.ax_joint_id
        point = JointTrajectoryPoint()
        point.positions = angles
        point.time_from_start = rospy.Duration(5)
        goal.trajectory.points.append(point)
        self.jta.send_goal_and_wait(goal)

    def callback(self,msg):
        rospy.loginfo('Found joint trajectory action!')
        self.mx_joint_id = msg.id[0:3]
        self.ax_joint_id = msg.id[3:6]
        self.mx_joint_data = msg.data[0:3]
        self.ax_joint_data = msg.data[3:6]
        
        if self.name[0] == 'm':
            self.move_joint(self.mx_joint_data)  # mx
        else:
            self.move_joint(self.ax_joint_data)  # ax        

        # print self.mx_joint_data
        # print self.mx_joint_id
        #
        # print self.ax_joint_data
        # print self.ax_joint_id


def main():

    arm_m = Joint('m_arm')
    arm_a = Joint('a_arm')
    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('joint_position_tester')
    main()
