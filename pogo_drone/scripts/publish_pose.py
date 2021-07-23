#!/usr/bin/env python

import rospy
import tf
import math
import time
from geometry_msgs.msg import PoseStamped

if __name__ == '__main__':
    rospy.init_node('publish_pose', anonymous=True)
    worldFrame = rospy.get_param("~worldFrame", "/world")
    name = rospy.get_param("~name")
    r = rospy.get_param("~rate")
    x = rospy.get_param("~x")
    y = rospy.get_param("~y")
    z = rospy.get_param("~z")

    rate = rospy.Rate(r)

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = worldFrame

    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = z

    quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    pub = rospy.Publisher(name, PoseStamped, queue_size=1)

    t = rospy.Time.now().to_sec()
    val = 0

    while not rospy.is_shutdown():
        msg.header.seq += 1
        msg.header.stamp = rospy.Time.now()
        if rospy.Time.now().to_sec() - t > 20:
            msg.pose.position.z = 0.3 * math.cos(val*0.08) + .4 #go up and down along the z axis
            val += 1

        pub.publish(msg)
        rate.sleep()
