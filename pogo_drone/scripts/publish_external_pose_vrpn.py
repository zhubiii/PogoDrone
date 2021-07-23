#!/usr/bin/env python

import rospy
import tf
from geometry_msgs.msg import PoseStamped, TransformStamped
from crazyflie_driver.srv import UpdateParams, Takeoff, Land

def onNewTransform(transform):
    global msg
    global pub
    global firstTransform

    if firstTransform:
        # initialize kalman filter
        rospy.set_param("kalman/initialX", transform.pose.position.x)
        rospy.set_param("kalman/initialY", transform.pose.position.y)
        rospy.set_param("kalman/initialZ", transform.pose.position.z)
        update_params(["kalman/initialX", "kalman/initialY", "kalman/initialZ"])

        rospy.set_param("kalman/resetEstimation", 1)
        # rospy.set_param("locSrv/extPosStdDev", 1e-4)
        update_params(["kalman/resetEstimation"]) #, "locSrv/extPosStdDev"])
        firstTransform = False
    else:
        msg.header.frame_id = transform.header.frame_id
        msg.header.stamp = transform.header.stamp
        msg.header.seq += 1
        msg.pose.position.x = transform.pose.position.x
        msg.pose.position.y = transform.pose.position.y
        msg.pose.position.z = transform.pose.position.z
        msg.pose.orientation.x = transform.pose.orientation.x
        msg.pose.orientation.y = transform.pose.orientation.y
        msg.pose.orientation.z = transform.pose.orientation.z
        msg.pose.orientation.w = transform.pose.orientation.w
        pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('publish_external_position_vicon', anonymous=True)
    topic = rospy.get_param("~topic", "/vrpn_client_node/PogoDrone/pose")

    rospy.wait_for_service('update_params')
    rospy.loginfo("found update_params service")
    update_params = rospy.ServiceProxy('update_params', UpdateParams)

    firstTransform = True

    msg = PoseStamped()
    msg.header.seq = 0
    msg.header.stamp = rospy.Time.now()

    pub = rospy.Publisher("external_pose", PoseStamped, queue_size=1)
    rospy.Subscriber(topic, PoseStamped, onNewTransform)

    rospy.spin()
