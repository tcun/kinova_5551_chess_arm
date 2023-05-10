#!/usr/bin/env python3

# Import required modules
import rospy
import yaml
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from cv2 import aruco

class ArucoDetector:
    def __init__(self):
        rospy.init_node('aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.marker_pub = rospy.Publisher("/aruco_marker_positions", Float64MultiArray, queue_size=100)

        # Initialize the ArUco dictionary and parameters
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        self.aruco_params = aruco.DetectorParameters_create()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.aruco_params.adaptiveThreshWinSizeMin = 4
        self.aruco_params.adaptiveThreshWinSizeStep = 3
        self.aruco_params.minMarkerPerimeterRate = 0.05

        # Load the camera calibration file
        camera_info_file = "/home/tcun/.ros/camera_info/head_camera.yaml"
        with open(camera_info_file, 'r') as f:
            camera_info = yaml.safe_load(f)

        # Extract the intrinsic parameters and distortion coefficients
        self.camera_matrix = np.array(camera_info['camera_matrix']['data']).reshape(3, 3)
        self.dist_coeffs = np.array(camera_info['distortion_coefficients']['data'])

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
            return

        # Enhance the image for better marker detection
        cv_image = self.enhance_image(cv_image)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Estimate the pose of each marker and publish the positions
            self.publish_marker_positions(corners, ids)

        # Draw detected markers and display the image
        aruco.drawDetectedMarkers(cv_image, corners, ids)
        resized_image = cv2.resize(cv_image, (1024, 720))
        cv2.imshow("Aruco Detection", resized_image)
        cv2.waitKey(1)

    def enhance_image(self, cv_image):
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        h, s, v = cv2.split(hsv_image)
        green_channel = cv2.GaussianBlur(s, (3, 3), 0)

        weight = 0.4  # You can adjust this value
        enhanced_green_channel = cv2.addWeighted(green_channel, weight, gray_image, 1 - weight, 0)
        return cv2.addWeighted(gray_image, 0.5, enhanced_green_channel, 0.5, 0)

    def publish_marker_positions(self, corners, ids):
        last_known_positions = {}
        marker_positions = []

        # Load your camera's intrinsic parameters
        camera_matrix = self.camera_matrix
        dist_coeffs = self.dist_coeffs  # Update these values with your camera's distortion coefficients

        # Set the ArUco marker size in meters
        marker_size = 0.0145

        # Estimate the pose of each marker
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, dist_coeffs)

        for i in range(len(ids)):
            marker_id = ids[i][0]
            marker_position = np.mean(corners[i][0], axis=0)

            # Save the last known positions of the markers
            last_known_positions[marker_id] = (tvecs[i][0][0], tvecs[i][0][1], tvecs[i][0][2])
            marker_positions.append([marker_id, tvecs[i][0][0], tvecs[i][0][2], tvecs[i][0][2]])

        for marker_id, position in last_known_positions.items():
            if marker_id not in [mp[0] for mp in marker_positions]:
                marker_positions.append([marker_id, position[0], position[1], position[2]])

        # Prepare the marker positions array for publishing
        marker_positions_array = Float64MultiArray()
        marker_positions_array.layout.dim.append(MultiArrayDimension())
        marker_positions_array.layout.dim[0].size = len(marker_positions)
        marker_positions_array.layout.dim[0].stride = 4
        marker_positions_array.layout.dim[0].label = 'markers'
        marker_positions_array.data = np.array(marker_positions).flatten()

        # Publish the marker positions
        self.marker_pub.publish(marker_positions_array)

if __name__ == '__main__':
    try:
        ad = ArucoDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass