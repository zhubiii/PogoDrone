#!/usr/bin/env python

import rospy
import tf
import math
import time
import numpy as np
from crazyflie_driver.srv import TakeoffRequest, LandRequest, UpdateParams
from geometry_msgs.msg import PoseStamped

class PublishPose(object):
    def __init__(self):
        worldFrame = rospy.get_param("~worldFrame", "/world")
        name = rospy.get_param("~name")
        r = rospy.get_param("~rate")
        x = rospy.get_param("~x")
        y = rospy.get_param("~y")
        z = rospy.get_param("~z")
        init_delay = rospy.get_param("~init_delay", 15)
        delay = rospy.get_param("~delay", 5)
        min_actuation_height = rospy.get_param("~min_actuation_height", 0.05)
        floor_height = rospy.get_param("~floor_height", -2)

        self.x = x
        self.y = y
        self.rest_height = z # resting hover height of the drone
        self.z_pos = -1 # the height of the drone received by OptiTrack
        self.name = name
        self.worldFrame = worldFrame
        self.rate = r
        self.delay = delay # delay between pogo jumps
        self.init_delay = init_delay
        self.min_actuation_height = min_actuation_height #height at which the motors will reengage after dropping
        self.floor_height = floor_height # distance to which drone drops to for pogo jump

        self.pose_pub = rospy.Publisher(name, PoseStamped, queue_size=1)
        self.external_sub = rospy.Subscriber("external_pose", PoseStamped, self.updatePose)
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        self.init_t = rospy.Time.now().to_sec()
        self.t = rospy.Time.now().to_sec()
        self.period = rospy.Duration(1.0 / self.rate)
        self.timer = rospy.Timer(self.period, self.control_loop)

    def control_loop(self, event):
        msg = PoseStamped()
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.worldFrame

        quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
        msg.pose.orientation.x = quaternion[0]
        msg.pose.orientation.y = quaternion[1]
        msg.pose.orientation.z = quaternion[2]
        msg.pose.orientation.w = quaternion[3]
        
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()

        msg.pose.position.x = self.x
        msg.pose.position.y = self.y

        if (rospy.Time.now().to_sec() - self.init_t) > self.init_delay:
            if (rospy.Time.now().to_sec() - self.t) > self.delay:
                self.t = rospy.Time.now().to_sec()
                rospy.logwarn("Dropping!")
                rospy.set_param("pose/z", self.floor_height)
            elif self.z_pos < self.min_actuation_height:
                rospy.loginfo("Actuating!")
                rospy.set_param("pose/z", self.rest_height)

        msg.pose.position.z = rospy.get_param("~z")
        self.pose_pub.publish(msg)

    def updatePose(self, msg):
        self.z_pos = msg.pose.position.z

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    _ = PublishPose()
    rospy.spin()