import rospy
import numpy as np
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from numpy import fmod, pi, fabs, sin, cos

class FrameTranslation(object):
    """Switch between frames with the use of odom"""
    def __init__(self):
        rospy.loginfo("Init frame control")
        self.angle_sub = rospy.Subscriber("/imu_euler",Float32,self.angle_cb)
        self.orientation = 0.0

    def angle_cb(self,euler):
        self.orientation = euler.data
        # rospy.loginfo("orientation=%.2f", self.orientation)

    def kmt_world2local(self,twist_msg):
        new_msg = Twist()
        new_msg.linear.x = twist_msg.linear.x * np.cos(self.orientation) - twist_msg.linear.y * np.sin(self.orientation)
        new_msg.linear.y = twist_msg.linear.y * np.cos(self.orientation) + twist_msg.linear.x * np.sin(self.orientation)
        new_msg.angular.z = twist_msg.angular.z
        return new_msg

    def kmt_local2world(self,twist_msg):
        new_msg = Twist()
        new_msg.linear.x = twist_msg.linear.x * cos(self.orientation) + twist_msg.linear.y * sin(self.orientation)
        new_msg.linear.y = twist_msg.linear.y * cos(self.orientation) - twist_msg.linear.x * sin(self.orientation)
        new_msg.angular.z = twist_msg.angular.z
        return new_msg


class RotationCompesation(object):
    """Compensate the rotation with the use of odom"""
    def __init__(self):
        rospy.loginfo("Init rotation control")
        self.angle_sub = rospy.Subscriber("/imu_euler",Float32,self.angle_cb)
        self.orientation = 0.0
        self.target = 0.0

    def angle_cb(self,euler):
        self.orientation = euler.data
        # rospy.loginfo("orientation=%.2f", self.orientation)
        
    def compensate(self, twist_msg):
        self.target += twist_msg.angular.z * 0.01 # dt assuming 100Hz
        self.target = normalize_angle(self.target)
        new_msg = Twist()
        new_msg.linear.x = twist_msg.linear.x
        new_msg.linear.y = twist_msg.linear.y
        new_msg.angular.z = twist_msg.angular.z*0.8\
            + shortest_angular_distance(self.orientation,self.target)*2 #feed-forward + p-control
        # rospy.loginfo("in : x: %.2f, y: %.2f, th: %.2f"%(twist_msg.linear.x,twist_msg.linear.y,twist_msg.angular.z))        
        # rospy.loginfo("out: x: %.2f, y: %.2f, th: %.2f"%(new_msg.linear.x,new_msg.linear.y,new_msg.angular.z))
        return new_msg

# http://wiki.ros.org/angles
from numpy import fmod, pi, fabs

def normalize_angle_positive(angle):
    """ Normalizes the angle to be 0 to 2*pi
        It takes and returns radians. """
    return angle % (2.0*pi)

def normalize_angle(angle):
    """ Normalizes the angle to be -pi to +pi
        It takes and returns radians."""
    a = normalize_angle_positive(angle)
    if a > pi:
        a -= 2.0 *pi
    return a

def shortest_angular_distance(from_angle, to_angle):
    """ Given 2 angles, this returns the shortest angular
        difference.  The inputs and ouputs are of course radians.
 
        The result would always be -pi <= result <= pi. Adding the result
        to "from" will always get you an equivelent angle to "to".
    """
    return normalize_angle(to_angle-from_angle)