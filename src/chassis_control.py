import rospy
import numpy as np
from tf import transformations
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from numpy import fmod, pi, fabs, sin, cos

class FrameTranslation(object):
    """Switch between frames with the use of odom"""
    def __init__(self):
        rospy.loginfo("Init frame control")
        self.odom_sub = rospy.Subscriber("chassis_odom",Odometry,self.odom_cb)
        self.odom_data = None
        self.orientation = 0.0

    def odom_cb(self,odom_msg):
        self.odom_data = odom_msg
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w)
        self.orientation = transformations.euler_from_quaternion(quaternion)[2]
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
    def __init__(self, max_z_vel=3.0, kFF=0.8, kP=2.0):
        rospy.loginfo("Init rotation control")
        self.odom_sub = rospy.Subscriber("chassis_odom",Odometry,self.odom_cb)
        self.odom_data = None
        self.orientation = 0.0
        self.target = 0.0
        self.max_z_vel = max_z_vel
        self.kFF = kFF
        self.kP = kP

    def odom_cb(self,odom_msg):
        self.odom_data = odom_msg
        quaternion = (
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w)
        self.orientation = transformations.euler_from_quaternion(quaternion)[2]
        # rospy.loginfo("orientation=%.2f", self.orientation)

    def stop_z(self):
        self.target = self.orientation

    def compensate(self, twist_msg):
        self.target += twist_msg.angular.z * 0.01 # dt assuming 100Hz
        self.target = normalize_angle(self.target)
        new_msg = Twist()
        new_msg.linear.x = twist_msg.linear.x
        new_msg.linear.y = twist_msg.linear.y
        angle_diff = shortest_angular_distance(self.orientation,self.target)
        compensation = angle_diff if abs(angle_diff) < 1.0 else np.copysign(1.0,angle_diff)
        new_msg.angular.z = twist_msg.angular.z*self.kFF+compensation*self.kP
        new_msg.angular.z = new_msg.angular.z if abs(new_msg.angular.z)<self.max_z_vel\
            else np.copysign(self.max_z_vel,new_msg.angular.z)
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