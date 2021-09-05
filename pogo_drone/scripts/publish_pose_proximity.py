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
        return_height = rospy.get_param("~return_height", 1.75)
        slope_down_return_height = rospy.get_param("~slope_down_return_height", 0.025)
        hang_time = rospy.get_param("~hang_time", 0.25)
        radius = rospy.get_param("~radius", 0.1) 
        min_actuation_height = rospy.get_param("~min_actuation_height", 0.04)
        floor_height = rospy.get_param("~floor_height", -3.5)
        safe_threshold = rospy.get_param("~safe_threshold", 0.001)

        self.x = x
        self.y = y
        self.z = z # resting hover height of the drone
        self.return_height = return_height # we want to set the height higher than steady state hover so we avoid energy loss in bounce
        self.slope_down_return_height = slope_down_return_height
        self.hang_time = hang_time
        self.first_bounce = True
        self.go_high = False # switch to initiate return height waypoint
        self.x_pos = -1
        self.y_pos = -1
        self.z_pos = -1 # the height of the drone received by OptiTrack
        self.x_orientation = -1
        self.y_orientation = -1
        self.safe = False # if attitude is stable safe will be True (handled in callback)
        self.radius = radius # spherical threshold in which drone should drop
        self.name = name
        self.worldFrame = worldFrame
        self.rate = r
        self.min_actuation_height = min_actuation_height #height at which the motors will reengage after dropping
        self.floor_height = floor_height # distance to which drone drops to for pogo jump
        self.safe_threshold = safe_threshold

        self.pose_pub = rospy.Publisher(name, PoseStamped, queue_size=1)
        self.external_sub = rospy.Subscriber("external_pose", PoseStamped, self.updatePose)

        self.period = rospy.Duration(1.0 / self.rate)
        self.timer = rospy.Timer(self.period, self.control_loop)
        #self.time_instance = rospy.Time.now().to_sec()
        #self.time_start = rospy.Time.now().to_sec()
        #self.old_time = 0
        self.start_bounce = 0

        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

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

        # so that we only bounce once and then just return to hover
        if self.first_bounce:
            if (self.x_pos-self.x )**2 + (self.y_pos-self.y)**2 + (self.z_pos-self.z)**2 < self.radius**2 and self.safe: 
                rospy.logwarn("Dropping!")
                rospy.set_param("pose/z", self.floor_height)
                self.go_high = True
            elif self.z_pos < self.min_actuation_height and self.go_high:
                #rospy.loginfo("Actuating!")
                rospy.set_param("pose/z", self.return_height)
                self.first_bounce = False
                self.bounce = True
                self.start_bounce = rospy.Time.now().to_sec()

            # We want the waypoint to be set much higher on the after-bounce so that we achieve enough thrust to
            # continue going upward. After the drone goes into the sphere z-radius we reset waypoint to actual z
        elif self.bounce:
            if self.z_pos > self.z:
                rospy.set_param("pose/z", self.z)
                self.bounce = False
                self.go_high = False
                self.first_bounce = True # make false if only want one bounce
            elif rospy.Time.now().to_sec() - self.start_bounce > self.hang_time:
                z = rospy.get_param("pose/z")
                rospy.set_param("pose/z", z-self.slope_down_return_height) #gradually decrease return height 
            

        # update time instance for firmware
        #new_time = rospy.Time.now().to_sec()
        #self.time_instance = new_time - self.time_start
        #if self.time_instance - self.old_time > 0.1:
            #self.old_time = self.time_instance
            #rospy.set_param("ctrlMel/time_instance", self.time_instance)
            #self._update_params(["ctrlMel/time_instance"])

        msg.pose.position.z = rospy.get_param("~z")
        self.pose_pub.publish(msg)

    def updatePose(self, msg):
        self.x_pos = msg.pose.position.x
        self.y_pos = msg.pose.position.y
        self.z_pos = msg.pose.position.z
        
        changex = msg.pose.orientation.x-self.x_orientation
        changey = msg.pose.orientation.y-self.y_orientation
        level = False
        if abs(msg.pose.orientation.x) < 0.02 and abs(msg.pose.orientation.y) < 0.02:
            level = True
        if abs(changex) < self.safe_threshold and abs(changey) < self.safe_threshold and level:
            self.safe = True
        else:
            self.safe = False
            #rospy.logerr("UNSAFE")

        self.x_orientation = msg.pose.orientation.x
        self.y_orientation = msg.pose.orientation.y

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    _ = PublishPose()
    rospy.spin()