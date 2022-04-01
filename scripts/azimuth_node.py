#!/usr/bin/env python3
"""
Node for publishing azimuth (yaw) measured by IMU; relative to base_link frame.
"""
from re import X
import rospy
import tf2_ros
import tf2_msgs.msg
import std_msgs
from sensor_msgs.msg import Imu
import stamped_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_multiply

from numpy import deg2rad,rad2deg
from math import pi

class Quaternion():
    """ Quaternion vector. """
    def __init__(self,x=0,y=0,z=0,w=0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    
    def __arr__(self):
        """ Represent quaternion as array. """
        return [self.x, self.y, self.z, self.w]
    
    def __mul__(self,other):
        """ Quaternion multiplication. """
        new = quaternion_multiply(self.__arr__(), other.__arr__())
        new = Quaternion(new[0],new[1],new[2],new[3])
        return new
    
    def get_eul(self):
        """ Returns (roll, pitch, yaw) in radians. """
        return euler_from_quaternion(self.__arr__())        

    def get_yaw(self):
        """ Returns yaw. """
        eul = self.get_eul()
        return eul[2]

class Azimuth():
    """ Listen to IMU quaternion orientation. Tranfsorm to base_link frame. Publish yaw (=azimuth). """
    def __init__(self, correction_angle):
        self.q = Quaternion()
        self.correction_angle = correction_angle
        self.trans = None

        self.pub_azimuth = rospy.Publisher('azimuth', stamped_msgs.msg.Float64, queue_size=10)
        self.sub_orientation = rospy.Subscriber('imu/data', Imu, self.update_eul_vec, queue_size=10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

    def update_eul_vec(self, msg):
        """ Euler angles subscriber callback. """
        if self.trans:  # Check if transform has been looked up. If not do nothing.
            q_imu = Quaternion(msg.orientation.x, \
                            msg.orientation.y, \
                            msg.orientation.z, \
                            msg.orientation.w)

            q_baselink = Quaternion(self.trans.transform.rotation.x, \
                                    self.trans.transform.rotation.y, \
                                    self.trans.transform.rotation.z, \
                                    self.trans.transform.rotation.w)

            self.q = q_imu * q_baselink     # Quaternion multiplication (defined by class).

            pub_msg = stamped_msgs.msg.Float64()
            pub_msg.header = msg.header     # Copy header from IMU message (does not copy sequence).
            pub_msg.data = (self.q.get_yaw() + pi + self.correction_angle) % (2*pi)    # (-pi,pi) -> (0,2*pi) ... north --+corr--> 0 ... (corr,2*pi + corr) -> (0,2*pi)

            self.pub_azimuth.publish(pub_msg)

    def run(self):
        """ Run loop of object. """
        rate = rospy.Rate(10)
        while not self.trans:
            try:
                self.trans = self.tfBuffer.lookup_transform('imu', 'base_link', rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rate.sleep()
                continue
        rospy.spin()

def main():
    rospy.init_node('azimuth')

    correction_angle = float(rospy.get_param('~correction_angle', pi/2))   # Value to add to yaw for it to be north oriented.
    node = Azimuth(correction_angle)                            
    
    node.run()

if __name__ == '__main__':
    main()