#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point 
import sys
import math

rospy.init_node("curve_pub")
pub = rospy.Publisher("visualization_marker",Marker,queue_size=10)
rate = rospy.Rate(30)

L = 5.0
R = 10.0
ds = 0.10

while not rospy.is_shutdown():
    curve_data = Marker()
    curve_data.header.frame_id = "map"
    curve_data.header.stamp = rospy.Time.now()
    curve_data.ns = "curve"
    curve_data.id = 0

    curve_data.action = Marker.ADD
    curve_data.type = Marker.LINE_STRIP
    curve_data.scale.x = 0.1

    curve_data.color.r = 1.0
    curve_data.color.g = 0.0
    curve_data.color.b = 0.0
    curve_data.color.a = 1.0
    curve_data.pose.orientation.x=0.0
    curve_data.pose.orientation.y=0.0
    curve_data.pose.orientation.z=1.0
    curve_data.pose.orientation.w=0.0

    n = int(L / ds)
    dtheta = ds/R
    for i in range(n):
        p = Point()
        p.x = - R + R*math.cos(n*dtheta)
        p.y = - R + R*math.sin(n*dtheta)
        p.z = 0.0
        curve_data.points.append(p)

    pub.publish(curve_data)

    rate.sleep()