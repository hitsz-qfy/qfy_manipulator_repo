#!/usr/bin/env python
import roslib
roslib.load_manifest('qfy_dynamixel')

import rospy
import actionlib
from std_msgs.msg import Float64
import trajectory_msgs.msg 
import control_msgs.msg  
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryAction, JointTrajectoryGoal, FollowJointTrajectoryAction, FollowJointTrajectoryGoal



class Joint:
        def __init__(self, motor_name):
            #arm_name should be b_arm or f_arm
            self.name = motor_name

            self.jta = actionlib.SimpleActionClient('/'+self.name+'_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
            rospy.loginfo('Waiting for joint trajectory action')
            self.jta.wait_for_server()
            rospy.loginfo('Found joint trajectory action!')

            
        def move_joint(self, angles):
            goal = FollowJointTrajectoryGoal()                  
            #goal.trajectory.joint_names = ['joint_4', 'joint_5','joint_6']
            #goal.trajectory.joint_names = ['joint_3']
            if self.name[0] == 'm':
                goal.trajectory.joint_names = ['joint_1', 'joint_2','joint_3']
            else:
                goal.trajectory.joint_names = ['joint_4', 'joint_5','joint_6']
            point = JointTrajectoryPoint()
            point.positions = angles
            point.time_from_start = rospy.Duration(1)                   
            goal.trajectory.points.append(point)
            self.jta.send_goal_and_wait(goal)
              

def main():
            #arm = Joint('f_arm')
            #arm.move_joint([0.09, -1.6, 1.6])
            arm_m = Joint('m_arm')
            arm_a = Joint('a_arm')
            arm_a.move_joint([0.09,-1.6,0.6]) #ax
            arm_m.move_joint([2.0,1.0,4.0]) #mx

                        
if __name__ == '__main__':
      rospy.init_node('joint_position_tester')
      main()
