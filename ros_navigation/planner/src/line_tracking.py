#!/usr/bin/env python

# Author: Roger Pi Roig
import rospy

from nav_msgs.msg import Odometry
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped
# Transformations
from tf.transformations import euler_from_quaternion, quaternion_from_euler, quaternion_from_matrix

import copy

#This class publish the path of the robot and the actual uncertainty of the robot and the map.

#Topics to subscribe in RVIZ:

#   - /pose_ekf_slam/robot_tracking (splitted in 2 namespaces)
class tracking(object):
    # ========================================================================
    def __init__(self):

        #Subscribers to the odometry of the 2 filters (one using udpates, the other without)
        self.odom_sub = rospy.Subscriber('/mobile_base_controller/odom',Odometry, self.odom_callback)

        self.pub_track = rospy.Publisher('/robot_tracking',MarkerArray,queue_size = 1)

        #Robot paths to be published
        self.path1 = MarkerArray()
   
        print("Node initialized")

    # ===========================================================
  
    # ========================================================================
    # Get robot position information and publish its uncertainty (With updates)
    def odom_callback(self, data):


        #publish uncertainty
        p = data.pose.pose.position
        #publish track
        new_time = rospy.Time.now()
        if len(self.path1.markers) < 1:

            arrow = Marker()
            arrow.header.frame_id = "/odom"
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = "rob"
            arrow.action = Marker.ADD
            arrow.color.r = 1
            arrow.color.a = 0 #Not visible
            arrow.id = 1
            arrow.type = Marker.ARROW
            arrow.scale.x = 0.005
            arrow.scale.y = 0.01
            arrow.scale.z = 0.01
            arrow.points.append(p)
            arrow.points.append(p)
            self.path1.markers.append(arrow)

        elif new_time - self.path1.markers[-1].header.stamp > rospy.Duration(1): # 1 second

            arrow = Marker()
            arrow.header.frame_id = "/odom"
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = "rob"
            arrow.action = Marker.ADD
            arrow.color.r = 1
            arrow.color.a = 1 #visible
            arrow.id = self.path1.markers[-1].id +1
            arrow.type = Marker.ARROW
            arrow.scale.x = 0.005
            arrow.scale.y = 0.01
            arrow.scale.z = 0.01
            arrow.points.append(self.path1.markers[-1].points[1])
            arrow.points.append(p)
            self.path1.markers.append(arrow)

            self.pub_track.publish(self.path1)


  
# ========================================================================
if __name__ == '__main__':
    try:
        print("Starting")
        rospy.init_node('ekf_uncertainty_publisher', anonymous=True)
        t = tracking()
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

    except rospy.ROSInterruptException:
        pass


