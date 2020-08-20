#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker

from geometry_msgs.msg import Point 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
import tf2_ros
import sys
import math
import numpy as np
import transform as tfm

# xgoal = PoseStamped()
# xstart = PoseWithCovarianceStamped()

def callback_PoseStamped(data):
    rospy.loginfo(rospy.get_caller_id() + '%f', data.pose.position.x)
    xgoal = data

def callback_PoseWithCovarianceStamped(data):
    rospy.loginfo(rospy.get_caller_id() + '%f', data.pose.pose.position.y)
    xstart = data
    creat_curve(xstart)
    
def creat_curve(xstart):
    
    br = tf.TransformBroadcaster()
    quaternion = [xstart.pose.pose.orientation.x,xstart.pose.pose.orientation.y,xstart.pose.pose.orientation.z,xstart.pose.pose.orientation.w]
    e = tf.transformations.euler_from_quaternion(quaternion)
    RR = np.dot(tfm.Rz(-math.pi/2),tfm.rpy(e[0],e[1],e[2]))
    print(RR)
    br.sendTransform((xstart.pose.pose.position.x,xstart.pose.pose.position.y,xstart.pose.pose.position.z),quaternion,rospy.Time.now(),"robot","map")

    curve_data = Marker()
    curve_data.header.frame_id = "robot"
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
    # quaternion(wxyz)
    curve_data.pose.orientation.x=0.0
    curve_data.pose.orientation.y=0.0
    curve_data.pose.orientation.z=0.0
    curve_data.pose.orientation.w=1.0
    # curve_data.pose.orientation = xstart.pose.pose.orientation
    print( xstart.pose.pose.orientation)
    n = int(L / ds)
    dtheta = ds/R
    pp = np.zeros(3)
    for i in range(n):
        p = Point()
        pp[0] = - R + R*math.cos(i*dtheta)
        pp[1] = R*math.sin(i*dtheta)
        pp[2] = 0.0
        print(pp)
        # RR = tfm.Rz(-math.pi/2)
        # pp2 = np.dot(RR,pp.reshape(3,1))
        # p.x = xstart.pose.pose.position.x + pp2[0]
        # p.y = xstart.pose.pose.position.y + pp2[1]
        # p.z = pp2[2]
        # print(p)
        p.x = pp[0]
        p.y = pp[1]
        p.z = pp[2]
        curve_data.points.append(p)

    pub.publish(curve_data)


if __name__ == "__main__":
    
    args = sys.argv
    L = float(args[1])
    R = float(args[2])
    ds = 0.10

    rospy.init_node("curve_pub")
    pub = rospy.Publisher("visualization_marker",Marker,queue_size=10)
    rate = rospy.Rate(30)
    rospy.Subscriber("move_base_simple/goal", PoseStamped, callback_PoseStamped)
    rospy.Subscriber("initialpose", PoseWithCovarianceStamped, callback_PoseWithCovarianceStamped)
    
    
    rospy.spin()