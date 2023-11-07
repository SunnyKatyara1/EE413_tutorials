#!/usr/bin/env python3
import rospy
from std_srvs.srv import SetBool, SetBoolResponse
from std_msgs.msg import Float64

class GripperController:
    def __init__(self):
        self.left_pub = rospy.Publisher('/left_finger_controller/command', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/right_finger_controller/command', Float64, queue_size=10)
        self.srv = rospy.Service('toggle_gripper', SetBool, self.handle_toggle_gripper)
        self.open_position = 0.05
        self.close_position = 0.0

    def handle_toggle_gripper(self, req):
        if req.data:  # open the gripper
            self.left_pub.publish(self.open_position)
            self.right_pub.publish(-self.open_position)
        else:  # close the gripper
            self.left_pub.publish(self.close_position)
            self.right_pub.publish(self.close_position)
        return SetBoolResponse(success=True, message="Gripper action completed.")

if __name__ == '__main__':
    rospy.init_node('gripper_controller')
    controller = GripperController()
    rospy.spin()

