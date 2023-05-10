import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import tf2_ros as tf
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_geometry_msgs import PoseStamped
from math import pi
from std_srvs.srv import Empty
import copy
import numpy as np
import os
from std_msgs.msg import Float64MultiArray


class PickAndPlace(object):
  """PickAndPlace"""
  def __init__(self):
    # TO AVOID ISSUES WITH NAMESPACES
    os.system ("export ROS_HOSTNAME='/my_gen3_lite/'")
    # Initialize the node
    super(PickAndPlace, self).__init__()
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('example_move_it_trajectories')

    try:
      self.is_gripper_present = rospy.get_param("/my_gen3_lite/is_gripper_present", False)
      if self.is_gripper_present:
        print('gripper present!')
        gripper_joint_names = rospy.get_param("/my_gen3_lite/gripper_joint_names", [])
        self.gripper_joint_name = gripper_joint_names[0]
      else:
        print('gripper not present!')
        gripper_joint_name = ""
      self.degrees_of_freedom = rospy.get_param("/my_gen3_lite/degrees_of_freedom", 7)

      # Create the MoveItInterface necessary objects
      arm_group_name = "arm"
      self.robot = moveit_commander.RobotCommander("/my_gen3_lite/robot_description")
      self.scene = moveit_commander.PlanningSceneInterface(ns="/my_gen3_lite/")
      self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, robot_description = "/my_gen3_lite/robot_description", ns="/my_gen3_lite/")
      self.display_trajectory_publisher = rospy.Publisher('/my_gen3_lite/move_group/display_planned_path',
                                                    moveit_msgs.msg.DisplayTrajectory,
                                                    queue_size=20)
      self.chess_sub = rospy.Subscriber("/aruco_marker_positions", Float64MultiArray, self.chess_pieces_callback)
      self.chess_piece_positions = []
      if self.is_gripper_present:
        gripper_group_name = "gripper"
        self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, robot_description = "/my_gen3_lite/robot_description",ns="/my_gen3_lite/")

      # DISPLAY ROBOT INFORMATION
      # We can get the name of the reference frame for this robot:
      planning_frame = self.arm_group.get_planning_frame()
      print("============ Planning frame: %s" % planning_frame)
      # We can also print the name of the end-effector link for this group:
      eef_link = self.arm_group.get_end_effector_link()
      print("============ End effector link: %s" % eef_link)
      # We can get a list of all the groups in the robot:
      #group_names = self.arm_group.get_group_names()
      #print("============ Available Planning Groups:", self.robot.get_group_names())
      # Sometimes for debugging it is useful to print the entire state of the
      # robot:
      print("============ Printing robot state")
      print(self.robot.get_current_state())
      print("")
      rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())

    except Exception as e:
      print (e)
      self.is_init_success = False
    else:
      self.is_init_success = True

  def chess_pieces_callback(self, msg):
    # Initialize or clear the chess piece positions list
    self.chess_piece_positions = []
    
    # Process the incoming message data
    for i in range(0, len(msg.data)-3, 4):
        id = msg.data[i]
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "camera_link"  # Assuming this is the camera frame
        pose.pose.position.x = msg.data[i + 1]
        pose.pose.position.y = msg.data[i + 2]
        pose.pose.position.z = msg.data[i + 3]
        pose.pose.orientation.w = 1.0  # No orientation information

        cam_tf = self.get_camera_pose_in_base_frame()
        if cam_tf is not None: 
            pose_transformed = self.transform_pose(pose, cam_tf)
            if pose_transformed is not None:
                self.chess_piece_positions.append((id, pose_transformed))

  def transform_pose(self, pose, camera_to_base_link):
      if pose is None or camera_to_base_link is None:
          print("Pose error")
          return 

      # Convert the PoseStamped message to a geometry_msgs/TransformStamped message
      pose_transform = TransformStamped()
      pose_transform.header.stamp = rospy.Time.now()  # Adjust this as necessary
      pose_transform.header.frame_id = camera_to_base_link.header.frame_id
      pose_transform.child_frame_id = pose.header.frame_id
      pose_transform.transform.translation.x = pose.pose.position.x
      pose_transform.transform.translation.y = pose.pose.position.y
      pose_transform.transform.translation.z = pose.pose.position.z
      pose_transform.transform.rotation = pose.pose.orientation

      # Apply the camera to end effector transform
      pose_in_base_frame = tf2_geometry_msgs.do_transform_pose(pose, pose_transform)
      return pose_in_base_frame
    
  def get_camera_pose_in_base_frame(self):  
      # Create a tf2_ros.Buffer and a tf2_ros.TransformListener
      tf_buffer = tf.Buffer(rospy.Duration(1200.0))  # 1200 seconds buffer size
      tf_listener = tf.TransformListener(tf_buffer)

      # Define a PoseStamped for the camera pose in the end effector frame
      camera_pose_end_effector = PoseStamped()
      camera_pose_end_effector.header.stamp = rospy.Time.now()
      camera_pose_end_effector.header.frame_id = "gripper_base_link"

      # Set the translation
      camera_pose_end_effector.pose.position.x = 0.45
      camera_pose_end_effector.pose.position.y = 0
      camera_pose_end_effector.pose.position.z = 0.15

      # Set the orientation using the provided quaternion
      camera_pose_end_effector.pose.orientation.x = 0
      camera_pose_end_effector.pose.orientation.y = 0.425
      camera_pose_end_effector.pose.orientation.z = 0
      camera_pose_end_effector.pose.orientation.w = 0.905

      try:
          # Transform the camera pose from the end effector frame to the base frame
          camera_pose_base = tf_buffer.transform(camera_pose_end_effector, "base_link", rospy.Duration(1.0))
          return camera_pose_base
      except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
          return None

  def get_all_piece_positions(self):
      # This function returns all the chess piece positions
      return self.chess_piece_positions
    
  def get_piece_position(self, id):
      # This function returns the position of a specific chess piece based on its ID

      # Iterates through the list of tuples in self.chess_piece_positions
      for piece in self.chess_piece_positions:
          piece_id, pose = piece
          if piece_id == id:
              # Extracts the x, y, z coordinates from the pose
              x = pose.pose.position.x
              y = pose.pose.position.y
              z = pose.pose.position.z
              print(f"Piece with ID: {id} is at X: {x}, Y: {y}, Z: {z}")
              return x, y, z
      return None
  
  def set_pose_msg(self, pos_x,pos_y,pos_z,ori_x,ori_y,ori_z,ori_w):
    #set each parameter of a pose message msg.pose
    pose = geometry_msgs.msg.Pose()
    pose.position.x = pos_x
    pose.position.y = pos_y
    pose.position.z = pos_z
    
    pose.orientation.x = ori_x
    pose.orientation.y = ori_y
    pose.orientation.z = ori_z
    pose.orientation.w = ori_w    
      
    return pose

  def reach_named_position(self, target):
    arm_group = self.arm_group
    
    # Going to one of those targets
    rospy.loginfo("Going to named target " + target)
    # Set the target
    arm_group.set_named_target(target)
    # Plan the trajectory
    (success_flag, planned_path1, planning_time, error_code)= arm_group.plan()
    # Execute the trajectory and block while it's not finished
    return arm_group.execute(planned_path1, wait=True)


  def reach_joint_angles(self, config_pose, tolerance):
    arm_group = self.arm_group
    success = True

    # Get the current joint positions
    joint_positions = arm_group.get_current_joint_values()
    # rospy.loginfo("Printing current joint positions before movement :")
    for p in joint_positions: rospy.loginfo(p)

    # Set the goal joint tolerance
    self.arm_group.set_goal_joint_tolerance(tolerance)

    # Set the joint target configuration
    arm_group.set_joint_value_target(config_pose)
    
    # Plan and execute in one command
    success &= arm_group.go(wait=True)

    # Show joint positions after movement
    new_joint_positions = arm_group.get_current_joint_values()
    # rospy.loginfo("Printing current joint positions after movement :")
    for p in new_joint_positions: rospy.loginfo(p)
    return success

  def get_cartesian_pose(self):
    arm_group = self.arm_group

    # Get the current pose and display it
    pose = arm_group.get_current_pose()
    rospy.loginfo("Actual cartesian pose is : ")
    rospy.loginfo(pose.pose)

    return pose.pose

  def reach_cartesian_pose(self, pose, tolerance, constraints):
    arm_group = self.arm_group
    
    # Set the tolerance
    arm_group.set_goal_position_tolerance(tolerance)

    # Set the trajectory constraint if one is specified
    if constraints is not None:
      arm_group.set_path_constraints(constraints)

    # Get the current Cartesian Position
    arm_group.set_pose_target(pose)

    # Plan and execute
    rospy.loginfo("Planning and going to the Cartesian Pose")
    return arm_group.go(wait=True)

  def plan_cartesian_path(self, waypoints, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.arm_group

    ## BEGIN_SUB_TUTORIAL plan_cartesian_path

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher
    
    ## Displaying a Trajectory
    ## ^^^^^^^^^^^^^^^^^^^^^^^
    ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
    ## group.plan() method does this automatically so this is not that useful
    ## here (it just displays the same trajectory again):
    ##
    ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    ## We populate the trajectory_start with our current robot state to copy over
    ## any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory)

    ## END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.arm_group

    ## BEGIN_SUB_TUTORIAL execute_plan
    ##
    ## Executing a Plan
    ## ^^^^^^^^^^^^^^^^
    ## Use execute if you would like the robot to follow
    ## the plan that has already been computed:
    move_group.execute(plan, wait=True)

    ## **Note:** The robot's current joint state must be within some tolerance of the
    ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    ## END_SUB_TUTORIAL

  def reach_gripper_position(self, relative_position):
    gripper_group = self.gripper_group
    
    # We only have to move this joint because all others are mimic!
    gripper_joint = self.robot.get_joint(self.gripper_joint_name)
    gripper_max_absolute_pos = gripper_joint.max_bound()
    gripper_min_absolute_pos = gripper_joint.min_bound()
    try:
      val = gripper_joint.move(relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos, True)
      return val
    except:
      return False 