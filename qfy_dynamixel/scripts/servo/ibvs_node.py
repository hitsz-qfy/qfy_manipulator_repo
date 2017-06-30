#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import track_2
import tf
from qfy_dynamixel.srv import *

from dynamixel_msgs.msg import MultiJointCommand
from dynamixel_msgs.msg import MotorStateFloatList, MotorStateFloat
from dynamixel_driver.dynamixel_const import *
from qfy_dynamixel.msg import multi_joint_point, PbvsErrorStamped
from qfy_ibvs.msg import ImagePoint2D, FourImagePointStamped

class Ibvs(object):
    def __init__(self):
        self.tracker = track_2.Track()

        self.sub_apriltag = rospy.Subscriber('/points2D_info', FourImagePointStamped, self.apriltag_points_cb)

        self.interaction_mat = np.matrix(np.zeros((8,6)))
        self.error_mat = np.matrix(np.zeros((8,1)))

        self.camera_intrinsic_mat = np.asmatrix([[574.666883, 0.000000, 335.073855],
                                                 [0.000000, 574.765706, 247.464029],
                                                 [0.000000, 0.000000, 1.000000]])

        self.des_point0 = ImagePoint2D()
        self.des_point0.pixel.x = 227.446
        self.des_point0.pixel.y = 302.527
        vec_point0_pixel = np.asmatrix([self.des_point0.pixel.x, self.des_point0.pixel.y, 1])
        self.des_point0_metric_mat = self.camera_intrinsic_mat.I * vec_point0_pixel
        self.des_point0.normalized.x = self.des_point0_metric_mat.item(0)
        self.des_point0.normalized.y = self.des_point0_metric_mat.item(1)

        self.des_point1 = ImagePoint2D()
        self.des_point1.pixel.x = 379.885
        self.des_point1.pixel.y = 306.196
        vec_point1_pixel = np.asmatrix([self.des_point1.pixel.x, self.des_point1.pixel.y, 1])
        self.des_point1_metric_mat = self.camera_intrinsic_mat.I * vec_point1_pixel
        self.des_point1.normalized.x = self.des_point1_metric_mat.item(0)
        self.des_point1.normalized.y = self.des_point1_metric_mat.item(1)

        self.des_point2 = ImagePoint2D()
        self.des_point2.pixel.x = 388.910
        self.des_point2.pixel.y = 144.532
        vec_point2_pixel = np.asmatrix([self.des_point2.pixel.x, self.des_point2.pixel.y, 1])
        self.des_point2_metric_mat = self.camera_intrinsic_mat.I * vec_point2_pixel
        self.des_point2.normalized.x = self.des_point2_metric_mat.item(0)
        self.des_point2.normalized.y = self.des_point2_metric_mat.item(1)

        self.des_point3 = ImagePoint2D()
        self.des_point3.pixel.x = 228.434
        self.des_point3.pixel.y = 137.520
        vec_point3_pixel = np.asmatrix([self.des_point3.pixel.x, self.des_point3.pixel.y, 1])
        self.des_point3_metric_mat = self.camera_intrinsic_mat.I * vec_point3_pixel
        self.des_point3.normalized.x = self.des_point3_metric_mat.item(0)
        self.des_point3.normalized.y = self.des_point3_metric_mat.item(1)

        self.get_tf = False

    def calcu_error(self, points):
        self.error_mat.itemset(0, (points.p0.normalized.x - self.des_point0.normalized.x))
        self.error_mat.itemset(1, (points.p0.normalized.y - self.des_point0.normalized.y))
        self.error_mat.itemset(2, (points.p1.normalized.x - self.des_point1.normalized.x))
        self.error_mat.itemset(3, (points.p1.normalized.y - self.des_point1.normalized.y))
        self.error_mat.itemset(4, (points.p2.normalized.x - self.des_point2.normalized.x))
        self.error_mat.itemset(5, (points.p2.normalized.y - self.des_point2.normalized.y))
        self.error_mat.itemset(6, (points.p3.normalized.x - self.des_point3.normalized.x))
        self.error_mat.itemset(7, (points.p3.normalized.y - self.des_point3.normalized.y))

    def calcu_interaction_mat(self,points):
        i = 0
        while i < 8:
            self.interaction_mat.itemset((i,0), -1. / self.tracker.trans[2])
            self.interaction_mat.itemset((i,1), 0.)
            self.interaction_mat.itemset((i,2), points[i] / self.tracker.trans[2])
            self.interaction_mat.itemset((i,3), points[i] * points[i+1])
            self.interaction_mat.itemset((i,4), - (1 + points[i] * points[i]))
            self.interaction_mat.itemset((i,5), points[i + 1])

            i += 1

            self.interaction_mat.itemset((i, 0), 0.)
            self.interaction_mat.itemset((i, 1), -1. / self.tracker.trans[2])
            self.interaction_mat.itemset((i, 2), points[i] / self.tracker.trans[2])
            self.interaction_mat.itemset((i, 3), (1 + points[i] * points[i]))
            self.interaction_mat.itemset((i, 4), - points[i] * points[i - 1])
            self.interaction_mat.itemset((i, 5), - points[i - 1])

        return self.interaction_mat

    def apriltag_points_cb(self,msg):
        self.calcu_error(msg)
        if self.check_get_tf():
            cur_points_list = [msg.p0.normalized.x, msg.p0.normalized.y,
                               msg.p1.normalized.x, msg.p1.normalized.y,
                               msg.p2.normalized.x, msg.p2.normalized.y,
                               msg.p3.normalized.x, msg.p3.normalized.y]
            self.calcu_interaction_mat(cur_points_list)


    def check_get_tf(self):
        if self.tracker.get_now_tf():
            return self.tracker.get_tf
        else:
            return self.tracker.get_tf
