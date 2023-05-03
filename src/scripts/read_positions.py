#!/usr/bin/env python3

import rospy
from kinova_5551_chess_arm.srv import CaptureAndProcessImage
from kinova_5551_chess_arm.msg import ArucoPositions

def aruco_positions_callback(msg):
    rospy.loginfo('Received ArUco positions: {}'.format(msg.aruco_positions))

def main():
    rospy.init_node('call_service_and_read_positions')

    # Wait for the capture_and_process_image service to become available
    rospy.loginfo('Waiting for capture_and_process_image service...')
    rospy.wait_for_service('capture_and_process_image')

    # Create a client for the capture_and_process_image service
    capture_and_process_image_client = rospy.ServiceProxy('capture_and_process_image', CaptureAndProcessImage)

    # Call the capture_and_process_image service
    rospy.loginfo('Calling capture_and_process_image service...')
    response = capture_and_process_image_client()
    if response.success:
        rospy.loginfo('Service call successful: {}'.format(response.message))
    else:
        rospy.logerr('Service call failed: {}'.format(response.message))

    # Subscribe to the aruco_positions topic
    rospy.Subscriber('aruco_positions', ArucoPositions, aruco_positions_callback)

    # Spin the node to keep it running and processing callbacks
    rospy.spin()

if __name__ == '__main__':
    main()
