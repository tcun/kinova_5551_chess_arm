#!/usr/bin/env python3

import copy
import json
import sys
import time
import numpy as np
import math 
import geometry_msgs.msg
import rospy
from kinova_engine import PickAndPlace
from std_srvs.srv import Empty


#TO DO:
# add funtionality to chess board and chess piece classes
# add named position for calibration or for after moving

#######UNFINISHED#######

class ChessBoard(object):
    def __init__(self) -> None:
        #init chess board class
        # board_matrix = np.array([['r1_w', 'k1_w', 'b1_w','Q_w','K_w','b2_w','k2_w','r2_w'],
        #                          ['p_w','p_w','p_w','p_w','p_w','p_w','p_w','p_w'],
        #                          [None,None,None,None,None,None,None,None],
        #                          [None,None,None,None,None,None,None,None],
        #                          [None,None,None,None,None,None,None,None],
        #                          [None,None,None,None,None,None,None,None],
        #                          ['p_b','p_b','p_b','p_b','p_b','p_b','p_b','p_b'],
        #                          ['r1_b', 'k1_b', 'b1_b','Q_b','K_b','b2_b','k2_b','r2_b']])
    
        self.board_matrix = np.array([[210,212,214,216,215,213,211,209],
                                [208,207,206,205,204,203,202,201],
                                [0,0,0,0,0,0,0,0],
                                [0,0,0,0,0,0,0,0],
                                [0,0,0,0,0,0,0,0],
                                [0,0,0,0,0,0,0,0],
                                [301,302,303,304,305,306,307,308],
                                [309,311,313,315,316,314,312,310]])
        self.letter_to_num_map = {"a":0,"b":1,"c":2,
                            "d":3,"e":4,"f":5,
                            "g":6,"h": 7}
        # global orientation in x, y, theta
        self.global_orientation = .27, -.186, math.pi
        
        
        
    def update_board(self, pos_1, pos_2, kill_bool):
        def flip(pos_1, pos_2):
            temp = pos_1
            self.board_matrix[pos_1] = self.board_matrix[pos_2]
            self.board_matrix[pos_2] = self.board_matrix[temp]

        init_col = self.letter_to_num_map[pos_1[0]]
        init_row = int(pos_1[1]-1)
        end_col = self.letter_to_num_map[pos_2[2]]
        end_row = int(pos_2[3]-1)
        if kill_bool:
            self.board_matrix[end_row, end_col] = 0
        flip([init_row,init_col], [end_row, end_col])
        print(self.board_matrix)


    ############ UNUSED: Arm Movement Debug #####################    
    def calculate_global_orientation(self):
        
        ####### DEBUG ###### 
        board_x = 0.27 # 
        board_y = -.186
        theta = math.pi
        ########### UNCOMMENT AFTER DEBUG PHASE #######
        # Calculate global orientation of chess board based on Aruco marker positions
        # aruco_ids = [101, 102]
        # marker_positions = {aruco_id: None for aruco_id in aruco_ids}

        # for position in aruco_positions:
        #     if position.id in aruco_ids:
        #         marker_positions[position.id] = np.array([position.x, position.y, position.z])

        # # Calculation 
        # normal_101_102 = marker_positions[102] - marker_positions[101]
        
        # # Calculate angle between y_world and y_board
        # theta = math.atan2(normal_101_102[0], normal_101_102[1]) + math.pi

        # # Set origin coordinates for chess board
        # board_x = marker_positions[101][0]
        # board_y = marker_positions[101][1]

        # Set global orientation for chess board
        self.global_orientation = board_x, board_y, theta

    def get_square_coordinates(self, position):
        #given a board position return the x y world coordinate position
        print("the position we are calculating is:" , position)

        # Get the board orientation for calculations
        d_x = self.global_orientation[0]
        d_y = self.global_orientation[1]
        theta = self.global_orientation[2]
        
        print("The board orienation is x: ", d_x, "y: ", d_y, "theta: ", theta)
        #square size
        sq_inc = 0.03576

        #Right Now Sending XX will 
        board_letter_map = {"a":0.5*sq_inc,"b":1.5*sq_inc,"c":2.5*sq_inc,
                            "d":3.5*sq_inc,"e":4.5*sq_inc,"f":5.5*sq_inc,
                            "g":6.5*sq_inc,"h": 7.5*sq_inc }
        
        board_number_map = {"1":0.5*sq_inc,"2":1.5*sq_inc,"3":2.5*sq_inc,
                            "4":3.5*sq_inc,"5":4.5*sq_inc,"6":5.5*sq_inc,
                            "7":6.5*sq_inc,"8": 7.5*sq_inc}
        
        # vector maps letter to x coordinate and number to y coordinate
        square_vector = np.array([board_letter_map[position[0]], 
                                  board_number_map[position[1]], 
                                  0])

        # rotations will always be around the z axis
        R_wb = np.array([[math.cos(theta), -math.sin(theta), 0],
                        [math.sin(theta), math.cos(theta), 0], 
                        [0, 0, 1]])
        
        world_coord = np.matmul(R_wb, np.transpose(square_vector))
        print("The calculated board coorinate before adding world coordinates are x: ", world_coord[0], "y: ", world_coord[1])

        x_coordinate = world_coord[0] + d_x
        y_coordinate = world_coord[1] + d_y
        print("going to coordinate x: ", x_coordinate, "y: ",y_coordinate )


        return x_coordinate, y_coordinate

    def get_square_status(self, position):
        #given a board position string return piece number if piece exists there or return null
        print("getting status of position: ", position)
        idx_col = self.letter_to_num_map[position[0]]
        idx_row = int(position[1])-1

        piece_number = self.board_matrix[idx_row,idx_col]
        print("found ", piece_number, "at matrix position ", idx_row, " ", idx_col )
        
        return piece_number
    
    def get_graveyard_coordinates(self):
        x_coordinate = .3
        y_coordinate = -.3

        return x_coordinate, y_coordinate
    
    # def update_board(self):
    #     self.board_matrix = [[None for _ in range(8)] for _ in range(8)]  # Clear the board
    #     for i in range(0, len(self.chess_piece_positions), 4):  # Process in sets of 4 (id, x, y, z)
    #         piece_id = self.chess_piece_positions[i]
    #         piece_pos_camera_frame = self.chess_piece_positions[i+1:i+4]
            
    #         # Transform to world frame
    #         piece_pos_world_frame = self.transform_to_world_frame(piece_pos_camera_frame)
            
    #         # Find the nearest square
    #         square = self.find_nearest_square(piece_pos_world_frame)
            
    #         # Update the board matrix
    #         self.board_matrix[square[0]][square[1]] = piece_id


    # def find_nearest_square(self, piece_pos):
    #     piece_pos = np.array(piece_pos)
    #     min_distance = float('inf')
    #     min_idx = None
    #     for i in range(8):
    #         for j in range(8):
    #             distance = np.linalg.norm(piece_pos - np.array(self.chess_board[i][j]))
    #             if distance < min_distance:
    #                 min_distance = distance
    #                 min_idx = (i, j)
    #     return min_idx

    

##### UNFINISHED #######
class ChessPiece(object):
    def __init__(self) -> None:
        self.engine = PickAndPlace()
        self.chessboard = ChessBoard()
    
    def is_friendly(self, piece_number):
        # Check if piece is friendly (i.e. same team)
        friend_bool = False

        if piece_number >= 201 and piece_number<=216:
            friend_bool =True

        return friend_bool

    def get_world_coord(self, piece_number):

       ####### Do dumb stuff for the arm debug code#########
       ###### this is not how this should work once we have the camera####
       ###### pieces should understand their own position #####
        self.engine.get_piece_position(piece_number)

        index = np.where(self.chessboard.board_matrix == piece_number)
        num_to_letter = {0:"a", 1:"b", 2:"c", 
                         3:"d", 4:"e", 5:"f",
                         6:"g", 7:"h"}
        pos= num_to_letter[index[1][0]] + str((index[0][0] + 1))
        print("with debug method getting pirce from position", pos)

        x , y = self.chessboard.get_square_coordinates(position=pos)
        return x, y

class KinovaChessControl(object):
    def __init__(self) -> None:
        
        self.engine = PickAndPlace()
        self.board = ChessBoard()
        self.piece = ChessPiece()

        #Initialize gripper orientation to be straight up and down. 
        self.gripper_ori = {"ori_x": 0.7119706821454233, 
                             "ori_y": 0.7020579469382366, 
                             "ori_z": 0.010645562432992799, 
                             "ori_w": 0.009952834105663458}
        
        #Initialize gripper open and close closed joint positions
        """REPLACE WITH CUSTOM GRIPPER JOINT POSITIONS"""
        #open = 60% open
        #grab = 10% open 
        #close = 0% open 
        self.gripper_setting = {"open": 0.3, "grab":0.1, "close":0, "wide":1} 

        #Initialize heights for moving pieces around
        """Adjust Move and Pick Heights"""
        self.height_setting = {"over":0.15, "pick":0.005}

        # Get the global orientaion of the board
        self.board.calculate_global_orientation()

        # Initialize custom joint states [This could also be done with the UDRF]
        self.saved_joint_states = {"cali_1":[-0.1753041766511192, 0.22052864670230046, 
                                              2.013744464573454, -2.2268759150319504, 
                                              -1.268418886098428, 0.03974395084390652]}



    def get_move_from_user(self):
        first_move = True
        while True:
            try:
                if first_move: 
                    move = input("Input piece current position and desired position \n(e.g., c2c3 to move piece on c2 to c3) > ")
                else:
                    move = input("Try again > ")
                current_pos, final_pos, kill_bool = self.process_move(move)
                break

            except ValueError as e:
                print(e)
                first_move = False
        
        return current_pos, final_pos, kill_bool
        
    def process_move(self, move_string):
        #Function to take in movement string sanitize the input and check the move type.
        ##Sanitize input before parsing##
        #All of the letters on a board a through h.
        valid_letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
        valid_numbers = ['1', '2', '3', '4', '5', '6', '7', '8']
                         
        #Verify correct length
        if len(move_string) != 4:
            raise ValueError("Move Invalid: Input must be four characters long(e.g., a2a4)")

        #Verify first charector is a lower case letter between a and h 
        if not (move_string[0] in valid_letters):
            raise ValueError("Move Invalid: First character should be a lower case letter a-h (e.g., e2a4)")

        #Verify second charector is a digit 1-8
        if not (move_string[1] in valid_numbers):
            raise ValueError("Move Invalid: Second character should be digit 1-8 (e.g., a2a4)")

        #Verify third charector is a lower case letter between a and h 
        if not (move_string[2] in valid_letters):
            raise ValueError("Move Invalid: Third character should be a lower case letter a-h(e.g., a2a4)")

        #Verify fourth charector is a digit 1-8
        if not (move_string[3] in valid_numbers):
            raise ValueError("Move Invalid: Fourth character should be digit 1-8 (e.g., a2a4)")

        current_pos = move_string[:2]
        final_pos = move_string[2:]

        kill_move =self.validate_move_type(current_pos=current_pos, final_pos=final_pos)

        return current_pos, final_pos, kill_move
    
    def validate_move_type(self, current_pos, final_pos):
        #Given current and final positions validate the move and return if it is a kill
        #Verify current and final positions are not the same
        if current_pos == final_pos:
            raise ValueError("Move Invalid: First and second positions are the same")

        move_piece = self.board.get_square_status(current_pos)

        #Verify there is a friendly piece at the current position
        if not self.piece.is_friendly(move_piece):
            raise ValueError("Move Invalid: You don't have a piece at your first position")
         
        target_piece = self.board.get_square_status(final_pos)
        
        #Verify there is not a friendly piece at the final position 
        if self.piece.is_friendly(target_piece):
            raise ValueError("Move Invalid: Are you trying to sabotage yourself you have a piece at your goal position")

        #Check for kill move
        elif target_piece == 0:
            kill_move = False

        elif not self.piece.is_friendly(target_piece):
            kill_move = True

        return kill_move

    def move_gripper(self, pose):
        attempts = 5
        tolerance = 0.01
        for i in range(attempts):
            success = self.engine.reach_cartesian_pose(pose=pose, tolerance=tolerance, constraints=None)
            if success:
                return True
        
        rospy.loginfo("The gripper failed to plan even after {} times of trying.".format(attempts))
        return False 

    def execute_chess_move(self, current_pos, final_pos, kill_move):
        #Check move validity and type
        if kill_move:
            #remove the victim
            self.remove_piece(final_pos)

            self.board.update_board(current_pos, final_pos, kill_move)
            #move to empty location
            self.move_piece(current_pos, final_pos)

        else:
            #move to empty location
            self.move_piece(current_pos, final_pos)
            #self.board.update_board(current_pos, final_pos, False)
        
        self.reach_custom_joint_state("cali_1")
        self.engine.get_cartesian_pose()
        return 

    def move_piece(self, current_pos, final_pos):
        #get piece number at given board initial position
        piece = self.board.get_square_status(position=current_pos)
        #pickup the piece
        self.pick_piece(piece)
        #move the piece to the given final position
        self.place_piece(final_pos)

    def remove_piece(self, position):
        #get piece number at given board position
        piece = self.board.get_square_status(position=position)
        #pickup the piece
        self.pick_piece(piece)
        #throw the piece in the graveyard 
        self.trash_piece()

    def pick_piece(self, piece_number):
        success = True

        #Get Piece Coordinate
        x, y = self.piece.get_world_coord(piece_number)

        #set pose msg to above piece
        piece_location = self.engine.set_pose_msg(pos_x= x, 
                                                  pos_y=y, 
                                                  pos_z=self.height_setting["over"], 
                                                  ori_w=self.gripper_ori["ori_w"], 
                                                  ori_x=self.gripper_ori["ori_x"], 
                                                  ori_y=self.gripper_ori["ori_y"], 
                                                  ori_z=self.gripper_ori["ori_z"])
       
        #Alternatively get world coordinate could return a pose msg and only the height would need to be changes 
        # piece_location = self.piece.get_world_coord(piece_number) 
        # piece_location.position.z = elf.height_setting["over"]
        # piece_location = pose

        #move to xy location above piece
        success &= self.move_gripper(piece_location)
        #open gripper
        success &= self.engine.reach_gripper_position(self.gripper_setting["open"])
        #move down to pick-up height ----- may want to use the actual position of the gripper instead in case of discrepancies. 
        piece_location.position.z = self.height_setting["pick"]
        success &= self.move_gripper(piece_location)
        #close gripper (grab piece)
        success &= self.engine.reach_gripper_position(self.gripper_setting["grab"])
        #move to move height (pick up piece)
        piece_location.position.z = self.height_setting["over"]
        success &= self.move_gripper(piece_location)
        if success:
            print("picked up piece successfully!")
            self.engine.get_cartesian_pose()
        return success
    
    def place_piece(self, position):
        success =  True
        
        #get position coordinate
        x, y = self.board.get_square_coordinates(position=position)

        #set pose msg to above target location
        place_location = self.engine.get_cartesian_pose() #We assume here we just picked up a piece and are at movement height
        place_location.position.x = x
        place_location.position.y = y

        #move to above target square location
        success &= self.move_gripper(place_location)
        #move down to pick/place height
        place_location.position.z = self.height_setting["pick"]
        success &= self.move_gripper(place_location)
        #open the gripper
        success &= self.engine.reach_gripper_position(self.gripper_setting["open"])
        #move to above target square location
        place_location.position.z = self.height_setting["over"]
        success &= self.move_gripper(place_location)
        
        if success:
            print("placed piece successfully!")
            self.engine.get_cartesian_pose()
        return success

    def trash_piece(self):
        success = True

        #set pose to graveyard position
        x, y, z = self.board.get_graveyard_coordinates()
        grave_location = self.engine.set_pose_msg(pos_x= x, 
                                                  pos_y=y, 
                                                  pos_z=z, 
                                                  ori_w=self.gripper_ori["ori_w"], 
                                                  ori_x=self.gripper_ori["ori_x"], 
                                                  ori_y=self.gripper_ori["ori_y"], 
                                                  ori_z=self.gripper_ori["ori_z"])
        #move to grave position
        success &= self.move_gripper(grave_location)

        #open gripper
        success &= self.engine.reach_gripper_position(self.gripper_setting["open"])
        if success:
            print("Piece trashed successfully!")
        return success

    def user_calibration(self):
        success = True
        try:
            # go to where the board thinks the corner of the board is
            pose = self.engine.set_pose_msg(pos_x= self.board.global_orientation[0], 
                                                    pos_y=self.board.global_orientation[1], 
                                                    pos_z=self.height_setting["over"], 
                                                    ori_w=self.gripper_ori["ori_w"], 
                                                    ori_x=self.gripper_ori["ori_x"], 
                                                    ori_y=self.gripper_ori["ori_y"], 
                                                    ori_z=self.gripper_ori["ori_z"])
            print("Going to outtermost corner of square A1")
            print("pose message: ", pose)
            success &= self.move_gripper(pose)
            pose.position.z = self.height_setting["pick"]
            success &= self.move_gripper(pose)
            input("robot arm should be pointing at the outmost corner of the chess board press return to continue")
            pose.position.z = self.height_setting["over"]
            success &= self.move_gripper(pose)
            
            print("moving to position a8")
            #get position coordinate
            x, y = self.board.get_square_coordinates(position="a8")

            #set pose msg to above target location
            pose.position.x = x
            pose.position.y = y
            success &= self.move_gripper(pose)
            pose.position.z = self.height_setting["pick"]
            success &= self.move_gripper(pose)
            input("robot arm should be pointing to the center of H8 press return to continue")
            pose.position.z = self.height_setting["over"]
            success &= self.move_gripper(pose)
            print("moving to position 8")
            #get position coordinate
            x, y = self.board.get_square_coordinates(position="h8")

            #set pose msg to above target location
            pose.position.x = x
            pose.position.y = y
            success &= self.move_gripper(pose)
            pose.position.z = self.height_setting["pick"]
            success &= self.move_gripper(pose)
            input("robot arm should be pointing to the center of H8 press return to continue")
            pose.position.z = self.height_setting["over"]
            success &= self.move_gripper(pose)
            

            #get position coordinate
            print("moving to position A1")
            x, y = self.board.get_square_coordinates(position="a1")


            #set pose msg to above target location
            pose.position.x = x
            pose.position.y = y
            print("pose message: ", pose)
            success &= self.move_gripper(pose)
            pose.position.z = self.height_setting["pick"]
            success &= self.move_gripper(pose)
            input("robot arm should be pointing to the center of a1 press return to continue")
            
        except KeyboardInterrupt:
            print("Interrupted by keyboard. Stopping...")

    def reach_custom_joint_state(self, state_name):
        # get saved joint state
        joint_target = self.saved_joint_states[state_name]
        # set joint state
        self.engine.reach_joint_angles(joint_target, 0.01)


if __name__=="__main__":
    a = KinovaChessControl()
    a.reach_custom_joint_state("cali_1")
    
   
    a.engine.reach_gripper_position(a.gripper_setting["grab"])
    a.user_calibration()
    a.engine.reach_gripper_position(a.gripper_setting["wide"])
    print(a.engine.get_piece_position(205.0))
    # position = input("give a square potision(i.e. a2): " )
    a.board.get_square_coordinates("d3")
    # a.reach_custom_joint_state("cali_1")

    while True:
        try:
            user_input = input("Enter 'c' to continue testing, 'r' to run a different function, or 'q' to quit: ")
            
            if user_input == 'c':
                current_position, final_position, kill_bool = a.get_move_from_user()

                print("Current Position: ", current_position)
                print("Final Position: ", final_position)
                print("Kill Shot: ", kill_bool)

                a.execute_chess_move(current_position, final_position, kill_move= kill_bool)

            elif user_input == 'r':
                print("Select a function to run:")
                print("1. get_active_joints")
                print("2. get_current_joint_values")
                print("3. get_current_pose")
                print("4. get_current_rpy")
                print("5. get_end_effector_link")
                print("6. get_goal_joint_tolerance")
                print("7. get_goal_orientation_tolerance")
                print("8. get_goal_position_tolerance")
                print("9. get_goal_tolerance")
                print("10. get_joint_value_target")
                print("11. get_joints")
                print("12. get_known_constraints")

                selected_func = input("Enter the number of the function you want to run: ")
                try:
                    selected_func = int(selected_func)
                except ValueError:
                    print("Invalid input. Please enter a number.")
                    continue
                
                if selected_func == 1:
                    print(a.engine.arm_group.get_active_joints())
                elif selected_func == 2:
                    print(a.engine.arm_group.get_current_joint_values())
                elif selected_func == 3:
                    print(a.engine.arm_group.get_current_pose())
                elif selected_func == 4:
                    print(a.engine.arm_group.get_current_rpy())
                elif selected_func == 5:
                    print(a.engine.arm_group.get_end_effector_link())
                elif selected_func == 6:
                    print(a.engine.arm_group.get_goal_joint_tolerance())
                elif selected_func == 7:
                    print(a.engine.arm_group.get_goal_orientation_tolerance())
                elif selected_func == 8:
                    print(a.engine.arm_group.get_goal_position_tolerance())
                elif selected_func == 9:
                    print(a.engine.arm_group.get_goal_tolerance())
                elif selected_func == 10:
                    print(a.engine.arm_group.get_joint_value_target())
                elif selected_func == 11:
                    print(a.engine.arm_group.get_joints())
                elif selected_func == 12:
                    print(a.engine.arm_group.get_known_constraints())
                else:
                    print("Invalid input. Please enter a number between 1 and 12.")
                    continue

            elif user_input == 'q':
                print("Exiting the program...")
                break

            else:
                print("Invalid input. Please enter 'c', 'r', or 'q'.")
        except KeyboardInterrupt:
            print("\nReceived KeyboardInterrupt, exiting the program...")
            break
        except Exception as e:
            print("An error occurred:", e)



    ## Sanitize input
        ### success = true
        ### success &= len(input) == 4 -- check for 4 charectors 
        ### success &= -- make sure first is letter a b c d e f g h 
        ### success &= -- make sure second is number 1-8
        ### success &= -- make sure third is letter a b c d e f g h 
        ### success &= -- make sure second is number 1-8
        ###### if not success: print(input invalid)

    # Verify move
    # check if the desired piece is on the board. 
    ## verify_move(self, )
    ## ChessBoard.Status("c2") -- is one of my pieces there?
    ## if my piece is there continue 
    ## elseif no piece is there print(There's no piece there)
    ## elseif opponents piece is there print(That's not my piece)


    # Check if there is a friendly at the desired position == bad move
    # check if there is a enemy at the position == run kill move 
    # if there is no piece there  == run normal move 
    
