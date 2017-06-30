#!/usr/bin/env python
# coding=utf-8

import rospy
import tf2_ros
import tf
import math
from qfy_dynamixel.msg import multi_joint_point
# a1 = 7
# a2 = 246
# d4 = 110
# d6 = 111

#end-effector approach direction perpendicular to the object, the normal direction parallel to the object
def invekine(n,a,P,a1=7,a2=246,d4=110,d6=111):
    [nx,ny,nz]=n
    [ax,ay,az]=a
    [Px,Py,Pz]=P

    Px_ = Px - d6 * ax
    Py_ = Py - d6 * ay
    Pz_ = Pz - d6 * az

    ox = ay * nz - az * ny
    oy = -ax * nz + az * nx
    oz = ax * ny - ay * nx

    theta1 = math.atan2(Py_, Px_)

    cv = ((Px_ + a1 * math.cos(theta1)) ** 2 + (Py_ + a1 * math.sin(theta1)) ** 2 \
          + Pz_ ** 2 - a2 ** 2 - d4 ** 2) / (2 * a2 * d4)

    theta3 = math.atan2(math.sqrt(1 - cv ** 2), cv)

    theta2 = math.atan2(Pz_, math.sqrt((Px_ + a1 * math.cos(theta1)) ** 2 + (Py_ + a1 * math.sin(theta1)) ** 2)) \
             - math.atan2(d4 * math.sin(theta3), a2 + d4 * math.cos(theta3))

    theta4 = math.atan2(math.sin(theta1) * ax - math.cos(theta1) * ay, -math.cos(theta1) * math.sin(theta2 + theta3) \
                        * ax - math.sin(theta1) * math.sin(theta2 + theta3) * ay + math.cos(theta2 + theta3) * az)

    theta5 = math.acos(
        math.cos(theta1) * math.cos(theta2 + theta3) * ax + math.sin(theta1) * math.cos(theta2 + theta3) * ay \
        + math.sin(theta2 + theta3) * az)

    theta6 = math.atan2(
        math.cos(theta1) * math.cos(theta2 + theta3) * ox + math.sin(theta1) * math.cos(theta2 + theta3) * oy \
        + math.sin(theta2 + theta3) * oz, -math.cos(theta1) * math.cos(theta2 + theta3) * nx - math.sin(theta1) \
        * math.cos(theta2 + theta3) * ny - math.sin(theta2 + theta3) * nz)

    return [theta1, theta2, theta3, theta4, theta5, theta6]

def get_tf():
    tf2buffer = tf2_ros.Buffer()
    tf2listener = tf2_ros.TransformListener(tf2buffer)
    pub = rospy.Publisher('joint_goal_point', multi_joint_point, queue_size=10)

    pub_mjp = multi_joint_point()
    pub_mjp.id = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            trans = tf2buffer.lookup_transform('parent_frame', 'child_frame', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            continue

        position = trans.transform.translation
        quaternion = [trans.transform.rotation.x, trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w]
        rot3 = tf.transformations.quaternion_matrix(quaternion)
        ref_n = [rot3[0][0],rot3[1][0],rot3[2][0]]
        ref_a = [rot3[0][2],rot3[1][2],rot3[2][2]]
        ref_P = [position.x,position.y,position.z]
        joint_val = invekine(ref_n,ref_a,ref_P)

        pub_mjp.header.stamp = rospy.Time.now()
        pub_mjp.data = joint_val
        pub.publish(pub_mjp)
        # rospy.loginfo('translation [x y z]: [%d %d %d]' % (position.x,position.y, position.z))
        # rospy.loginfo('rotaion [nx ny nz]: [%f %f %f]' % (rot3[0][0],rot3[1][0],rot3[2][0]))
        # print a[:3][0]
        # rospy.loginfo('joint1 value: %f' % joint_val[0])
        # rospy.loginfo('joint2 value: %f' % joint_val[1])
        # rospy.loginfo('joint3 value: %f' % joint_val[2])
        # rospy.loginfo('joint4 value: %f' % joint_val[3])
        # rospy.loginfo('joint5 value: %f' % joint_val[4])
        # rospy.loginfo('joint6 value: %f' % joint_val[5])
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node("tf_invkine",anonymous=True)
    get_tf()

