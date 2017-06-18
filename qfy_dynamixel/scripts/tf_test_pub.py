#!/usr/bin/env python
import rospy

# Because of transformations
import tf

import tf2_ros
import geometry_msgs.msg
import PyKDL

def tf_broadcaster():
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    f = PyKDL.Frame( PyKDL.Rotation(0.3885,-0.6027,0.6970, -0.7672,-0.6305,-0.1177, 0.5104,-0.4890,-0.7073),
                        PyKDL.Vector(103.0246,-10.4905,-429.5767))
    #joint value q0=[0.1,-1.57,0.3,0.4,0.5,0.6]

    rot3 = f.M
    origin = f.p
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "parent_frame"
    t.child_frame_id = "child_frame"
    t.transform.translation.x = origin.x()
    t.transform.translation.y = origin.y()
    t.transform.translation.z = origin.z()
    [R,P,Y] = rot3.GetRPY()
    q = tf.transformations.quaternion_from_euler(R,P,Y)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)
    # rospy.loginfo('translation x: %d' % origin.x())
    # rospy.loginfo('translation y: %d' % origin.y())

if __name__ == '__main__':
    rospy.init_node("tf_broadcaster")
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        tf_broadcaster()
        rate.sleep()