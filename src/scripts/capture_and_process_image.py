#!/usr/bin/env python3
import rospy
import cv2
import os
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, TriggerResponse
from fiducial_msgs.msg import FiducialTransformArray
from kinova_5551_chess_arm.msg import ArucoPositions, ArucoPosition
from kinova_5551_chess_arm.srv import CaptureAndProcessImage, CaptureAndProcessImageResponse, ExportData, ExportDataResponse


class CaptureAndProcessImageNode:

    def __init__(self):
        rospy.init_node('capture_and_process_image_node')
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        self.aruco_sub = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.aruco_callback)
        self.aruco_positions_pub = rospy.Publisher('aruco_positions', ArucoPositions, queue_size=10)
        self.capture_and_process_image_service = rospy.Service('capture_and_process_image', CaptureAndProcessImage, self.handle_capture_and_process_image)
        self.export_data_service = rospy.Service('export_data', ExportData, self.capture_image_and_export_data)

    def image_callback(self, msg):
        # Store the latest image
        self.latest_image = msg

    def aruco_callback(self, msg):
        # Store the latest ArUco transforms
        self.latest_aruco_transforms = msg.transforms

    def capture_and_process_image(self, req):
        # Check if image and ArUco transforms have been received
        if not hasattr(self, 'latest_image'):
            return TriggerResponse(success=False, message='No image received yet')

        if not hasattr(self, 'latest_aruco_transforms'):
            return TriggerResponse(success=False, message='No ArUco transforms received yet')

        # Extract ArUco positions from transforms and publish them
        aruco_positions = [(t.fiducial_id, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z) for t in self.latest_aruco_transforms]
        self.aruco_positions_pub.publish(ArucoPositions(aruco_positions=aruco_positions))
        
        # Log the ArUco positions
        rospy.loginfo('ArUco positions: {}'.format(aruco_positions))

        return TriggerResponse(success=True, message='Image captured and ArUco positions retrieved successfully')
    
    def handle_capture_and_process_image(self, request):
        response = CaptureAndProcessImageResponse()
        response.success = True
        return response
    
    def capture_image_and_export_data(self, req):
        if not hasattr(self, 'latest_image'):
            return ExportDataResponse(success=False, message='No image received yet')

        if not hasattr(self, 'latest_aruco_transforms'):
            return ExportDataResponse(success=False, message='No ArUco transforms received yet')

        aruco_positions = [(t.fiducial_id, t.transform.translation.x, t.transform.translation.y, t.transform.translation.z) for t in self.latest_aruco_transforms]

        # Convert the ROS image to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')

        # Save the image to a file
        image_filename = os.path.join(os.path.expanduser('~'), 'aruco_image.png')
        cv2.imwrite(image_filename, cv_image)

        # Save the ArUco positions to a text file
        positions_filename = os.path.join(os.path.expanduser('~'), 'aruco_positions.txt')
        with open(positions_filename, 'w') as f:
            for pos in aruco_positions:
                f.write(f'{pos[0]} {pos[1]} {pos[2]} {pos[3]}\n')

        return ExportDataResponse(success=True, message=f'Image and positions exported to {image_filename} and {positions_filename}')

if __name__ == '__main__':
    # Create a CaptureAndProcessImageNode instance and start the ROS node
    capture_and_process_image_node = CaptureAndProcessImageNode()
    rospy.spin()
