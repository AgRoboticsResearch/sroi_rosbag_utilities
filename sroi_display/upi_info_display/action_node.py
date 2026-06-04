#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Bool


class ActionSetBoolServer:
    def __init__(self):
        self.is_action = False
        self.is_action_pub = rospy.Publisher('/upi/status/is_action', Bool, queue_size=10)
        self.toggle_srv = rospy.Service('/upi/status/toggle_srv', SetBool, self.set_bool)

    def set_bool(self, req):
        self.is_action = req.data
        print("Toggling is_action: ", self.is_action)
        return SetBoolResponse(success=True, message="Toggled is_action to " + str(self.is_action))

    def run(self):
        rospy.init_node('action_SetBool_server')
        rate = rospy.Rate(30) # 10hz

        print("Action SetBool Server is running")
        while not rospy.is_shutdown():
            self.is_action_pub.publish(self.is_action)
            rate.sleep()

        rospy.spin()


if __name__ == "__main__":
    action_SetBool_server = ActionSetBoolServer()
    action_SetBool_server.run()