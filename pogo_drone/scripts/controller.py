#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from crazyflie_driver.srv import UpdateParams
from std_srvs.srv import Empty

class Controller():
    def __init__(self, use_controller, joy_topic):
        rospy.wait_for_service('update_params')
        rospy.loginfo("found update_params service")
        self._update_params = rospy.ServiceProxy('update_params', UpdateParams)

        # mellenger controller
        rospy.set_param("stabilizer/controller", 2)
        self._update_params(["stabilizer/controller"])
        rospy.set_param("stabilizer/estimator", 2)
        self._update_params(["stabilizer/estimator"])

        ###XY
        #rospy.set_param("ctrlMel/kp_xy", .4) #default 0.4
        #self._update_params(["ctrlMel/kp_xy"])
        #rospy.set_param("ctrlMel/kd_xy", 0.2) #default 0.2
        #self._update_params(["ctrlMel/kd_xy"])
        #rospy.set_param("ctrlMel/ki_xy", 0.0) #default 0.05
        #self._update_params(["ctrlMel/ki_xy"])
        ### Z
        #rospy.set_param("ctrlMel/kp_z", 1.25) #default 1.25
        #self._update_params(["ctrlMel/kp_z"])
        #rospy.set_param("ctrlMel/kd_z", 0.4) #default 0.4
        #self._update_params(["ctrlMel/kd_z"])
        #rospy.set_param("ctrlMel/ki_z", 0.0) #default 0.05
        #self._update_params(["ctrlMel/ki_z"])

        # Attitude
        # 10 and 30 for P,D are good for shaking
        rospy.set_param("ctrlMel/kR_xy", 30000) #Pdefault: 70000 30000
        self._update_params(["ctrlMel/kR_xy"])
        rospy.set_param("ctrlMel/kw_xy", 40000) #Ddefualt: 20000 35000
        self._update_params(["ctrlMel/kw_xy"])
        rospy.set_param("ctrlMel/ki_m_xy", 0.0) #Idefualt: 0
        self._update_params(["ctrlMel/ki_m_xy"])

        rospy.set_param("ctrlMel/mass", 0.04)
        self._update_params(["ctrlMel/mass"])

        #rospy.set_param("ctrlMel/massThrust", 130000)
        #self._update_params(["ctrlMel/massThrust"])

        rospy.loginfo("waiting for emergency service")
        rospy.wait_for_service('emergency')
        rospy.loginfo("found emergency service")
        self._emergency = rospy.ServiceProxy('emergency', Empty)

        if use_controller:
            rospy.loginfo("waiting for land service")
            rospy.wait_for_service('land')
            rospy.loginfo("found land service")
            self._land = rospy.ServiceProxy('land', Empty)

            rospy.loginfo("waiting for takeoff service")
            rospy.wait_for_service('takeoff')
            rospy.loginfo("found takeoff service")
            self._takeoff = rospy.ServiceProxy('takeoff', Empty)
        else:
            self._land = None
            self._takeoff = None

        # subscribe to the joystick at the end to make sure that all required
        # services were found
        self._buttons = None
        rospy.Subscriber(joy_topic, Joy, self._joyChanged)

    def _joyChanged(self, data):
        for i in range(0, len(data.buttons)):
            if self._buttons == None or data.buttons[i] != self._buttons[i]:
                if i == 0 and data.buttons[i] == 1 and self._land != None:
                    self._land()
                if i == 1 and data.buttons[i] == 1:
                    self._emergency()
                if i == 2 and data.buttons[i] == 1 and self._takeoff != None:
                    self._takeoff()
                if i == 4 and data.buttons[i] == 1:
                    value = int(rospy.get_param("ring/headlightEnable"))
                    if value == 0:
                        rospy.set_param("ring/headlightEnable", 1)
                    else:
                        rospy.set_param("ring/headlightEnable", 0)
                    self._update_params(["ring/headlightEnable"])
                    print(not value)

        self._buttons = data.buttons

if __name__ == '__main__':
    rospy.init_node('crazyflie_demo_controller', anonymous=True)
    use_controller = rospy.get_param("~use_crazyflie_controller", False)
    joy_topic = rospy.get_param("~joy_topic", "joy")
    controller = Controller(use_controller, joy_topic)
    rospy.spin()
