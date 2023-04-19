#!/usr/bin/env python

import rospy
import actionlib
from kortex_driver.msg import (
    BaseCyclic_Feedback,
    Base_JointSpeeds,
    Base_JointAngles,
    Action,
    ActionType,
    GripperCommand,
    GripperMode
)

def main():
    rospy.init_node('gen3_lite_move_demo')

    # Initialize action client for controlling the gripper
    gripper_client = actionlib.SimpleActionClient('/my_gen3_lite/base/tool_IOs', ActionType)
    gripper_client.wait_for_server()

    # Initialize publisher for controlling the arm joints
    joint_speeds_publisher = rospy.Publisher('/my_gen3_lite/in/joint_speed', Base_JointSpeeds, queue_size=1)
    rospy.sleep(0.5)

    # Move gripper
    gripper_action = Action()
    gripper_action.name = "my_gripper_action"
    gripper_action.handle = 0
    gripper_action.input_mode = GripperMode.GRIPPER_POSITION
    gripper_action.gripper_command = GripperCommand.RELEASE
    gripper_client.send_goal(gripper_action)
    gripper_client.wait_for_result()

    # Move arm
    joint_speeds_msg = Base_JointSpeeds()
    joint_speeds_msg.joint_speeds.joint1 = 10.0
    joint_speeds_msg.joint_speeds.joint2 = 0.0
    joint_speeds_msg.joint_speeds.joint3 = 0.0
    joint_speeds_msg.joint_speeds.joint4 = 0.0
    joint_speeds_msg.joint_speeds.joint5 = 0.0
    joint_speeds_msg.joint_speeds.joint6 = 0.0
    joint_speeds_msg.joint_speeds.joint7 = 0.0

    duration = 3.0  # Move for 3 seconds
    rate = rospy.Rate(100)
    start_time = rospy.Time.now().to_sec()
    while (rospy.Time.now().to_sec() - start_time) < duration:
        joint_speeds_publisher.publish(joint_speeds_msg)
        rate.sleep()

    # Stop the arm
    joint_speeds_msg.joint_speeds.joint1 = 0.0
    joint_speeds_publisher.publish(joint_speeds_msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass