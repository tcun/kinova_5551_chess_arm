import rospy
import re
from ChessEnvironment import ChessBoard
from ChessEnvironment import ChessPiece
from KinovaEngine import PickAndPlace
from Kino.kinova_5551_chess_arm.msg import ArucoPositions, ArucoPosition
from kinova_5551_chess_arm.srv import GetMove, GetMoveResponse
from kinova_5551_chess_arm.srv import CaptureAndProcessImage
from kinova_5551_chess_arm.srv import CaptureAndProcessImageRequest


class KinovaChessControl(object):
    def __init__(self):

        # Initialize PickAndPlace, ChessBoard, and ChessPiece instances

        # !!!
        # TODO Make intailize ChessBoard and ChessPiece attributes using
        # !!!
        self.engine = PickAndPlace()
        self.board = ChessBoard()
        # self.piece = ChessPiece()

        # Set gripper orientation, gripper settings, and height settings
        self.gripper_ori = {"ori_x": 0.7119706821454233,
                            "ori_y": 0.7020579469382366,
                            "ori_z": 0.010645562432992799,
                            "ori_w": 0.009952834105663458}

        self.gripper_setting = {"open": 0.6, "grab": 0.3, "close": 0}
        self.height_setting = {"over": 0.15, "pick": 0.035}

        self.aruco_positions_received = False
        self.aruco_positions = None
        self.aruco_positions_sub = rospy.Subscriber(
            'aruco_positions', ArucoPositions, self.aruco_positions_callback)
        self.get_move_service = rospy.Service(
            'get_move', GetMove, self.handle_get_move)

    # def get_move_from_user(self):
    #     first_move = True
    #     while True:
    #         try:
    #             if first_move:
    #                 move = input(
    #                     "Input piece current position and desired position \n(e.g., c2c3 to move piece on c2 to c3) > ")
    #             else:
    #                 move = input("Try again > ")
    #             current_pos, final_pos = self.process_move(move)
    #             break

    #         except ValueError as e:
    #             print(e)
    #             first_move = False

    #     return current_pos, final_pos

    def handle_get_move(self, req):
        try:
            move = input(req.input_prompt)
            p_move = self.process_move(move)
            return GetMoveResponse(current_pos=p_move["current_pos"], final_pos=p_move["final_pos"], kill_move=p_move["kill_move"])
        except ValueError as e:
            print(f"Invalid move: {e}")
            return GetMoveResponse(current_pos="", final_pos="", kill_move="")

    def notation_to_index(self, notation):
        col = ord(notation[0]) - ord('a')
        row = int(notation[1]) - 1
        return row, col

    def process_move(self, move_string):
        # Function to take in movement string, sanitize the input, and check the move type.
        move_pattern = re.compile(r'^([a-h][1-8]){2}$')

        if not move_pattern.match(move_string):
            raise ValueError(
                "Move Invalid: Input must be in the format [a-h][1-8][a-h][1-8] (e.g., a2a4)")

        current_pos = self.notation_to_index(move_string[:2])
        final_pos = self.notation_to_index(move_string[2:])

        current_piece = self.board[current_pos[0], current_pos[1]]
        final_piece = self.board[final_pos[0], final_pos[1]]

        if not current_piece:
            return None

        # move_validity = self.validate_move(current_piece, final_pos)
        # if not move_validity["valid"]:
        #     return None

        kill_move = False
        if final_piece:
            if final_piece.team_type == current_piece.team_type:
                return None
            kill_move = True

        return {"current_pos": current_pos, "final_pos": final_pos, "kill_move": kill_move}

    # def process_move(self, move_string):
    #     # Function to take in movement string sanitize the input and check the move type.
    #     ## Sanitize input before parsing##
    #     # All of the letters on a board a through h.
    #     valid_letters = ['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h']
    #     valid_numbers = ['1', '2', '3', '4', '5', '6', '7', '8']

    #     # Verify correct length
    #     if len(move_string) != 4:
    #         raise ValueError(
    #             "Move Invalid: Input must be four characters long(e.g., a2a4)")

    #     # Verify first charector is a lower case letter between a and h
    #     if ~(move_string[0] in valid_letters):
    #         raise ValueError(
    #             "Move Invalid: First character should be a lower case letter a-h (e.g., e2a4)")

    #     # Verify second charector is a digit 1-8
    #     if ~(move_string[1] in valid_numbers):
    #         raise ValueError(
    #             "Move Invalid: Second character should be digit 1-8 (e.g., a2a4)")

    #     # Verify third charector is a lower case letter between a and h
    #     if ~(move_string[2] in valid_letters):
    #         raise ValueError(
    #             "Move Invalid: Third character should be a lower case letter a-h(e.g., a2a4)")

    #     # Verify fourth charector is a digit 1-8
    #     if ~(move_string[3] in valid_numbers):
    #         raise ValueError(
    #             "Move Invalid: Fourth character should be digit 1-8 (e.g., a2a4)")

    #     current_pos = move_string[:2]
    #     final_pos = move_string[2:]

    #     kill_move = self.validate_move_type(
    #         current_pos=current_pos, final_pos=final_pos)

    #     return current_pos, final_pos, kill_move

    def validate_move_type(self, current_pos, final_pos):
        # Given current and final positions validate the move and return if it is a kill
        # Verify current and final positions are not the same
        if current_pos == final_pos:
            raise ValueError(
                "Move Invalid: First and second positions are the same")

        move_piece = self.board.get_square_status(current_pos)

        # Verify there is a friendly piece at the current position
        if ~self.piece.is_friendly(move_piece):
            raise ValueError(
                "Move Invalid: You don't have a piece at your first position")

        target_piece = self.board.get_square_status(final_pos)

        # Verify there is not a friendly piece at the final position
        if self.piece.is_friendly(target_piece):
            raise ValueError(
                "Move Invalid: Are you trying to sabotage yourself you have a piece at your goal position")

        # Check for kill move
        if target_piece == None:
            kill_move = False

        if ~self.piece.is_friendly(target_piece):
            kill_move = True

        return kill_move

    def move_gripper(self, pose, attempts):

        tolerance = 0.01
        for i in range(attempts):
            success = self.engine.reach_cartesian_pose(
                pose=pose, tolerance=tolerance, constraints=None)
            if success:
                return True

        rospy.loginfo(
            "The gripper failed to plan even after {} times of trying.".format(attempts))
        return False

    def execute_move(self, current_pos, final_pos, kill_move):
        if kill_move:
            self.remove_piece(final_pos)
        self.move_piece(current_pos, final_pos)

        # Reach the named position
        self.engine.reach_named_position()

    def move_piece(self, current_pos, final_pos):
        # get piece number at given board initial position
        piece = self.board.get_square_status(position=current_pos)

        if piece is None:
            raise ValueError(
                "Invalid Piece: No Piece at board position")

        # pickup the piece
        self.pick_piece(piece)
        # move the piece to the given final position
        self.place_piece(final_pos)

    def remove_piece(self, position):
        # get piece number at given board position
        piece = self.board.get_square_status(position=position)

        if piece is None:
            raise ValueError(
                "Invalid Piece: No Piece at board position")
        # pickup the piece
        self.pick_piece(piece)
        # throw the piece in the graveyard
        self.trash_piece()

    def pick_piece(self, piece):
        success = True

        # Get Piece Coordinate
        x, y = piece.get_world_coord()

        # set pose msg to above piece
        piece_location = self.engine.set_pose_msg(pos_x=x,
                                                  pos_y=y,
                                                  pos_z=self.height_setting["over"],
                                                  ori_w=self.gripper_ori["ori_w"],
                                                  ori_x=self.gripper_ori["ori_x"],
                                                  ori_y=self.gripper_ori["ori_y"],
                                                  ori_z=self.gripper_ori["ori_z"])

        # Alternatively get world coordinate could return a pose msg and only the height would need to be changes
        # piece_location = self.piece.get_world_coord(piece_number)
        # piece_location.position.z = elf.height_setting["over"]
        # piece_location = pose

        # move to xy location above piece
        success &= self.move_gripper(piece_location)
        # open gripper
        success &= self.engine.reach_gripper_position(
            self.gripper_setting["open"])
        # move down to pick-up height ----- may want to use the actual position of the gripper instead in case of discrepancies.
        piece_location.position.z = self.height_setting["pick"]
        success &= self.move_gripper(piece_location)
        # close gripper (grab piece)
        success &= self.engine.reach_gripper_position(
            self.gripper_setting["grab"])
        # move to move height (pick up piece)
        piece_location.position.z = self.height_setting["over"]
        success &= self.move_gripper(piece_location)
        if success:
            print("picked up piece successfully!")
        return success

    def place_piece(self, board_position):
        success = True

        # get position coordinate
        x, y = self.board.get_square_coordinates(board_position=board_position)

        # set pose msg to above target location
        # We assume here we just picked up a piece and are at movement height
        place_location = self.engine.get_cartesian_pose()
        place_location.position.x = x
        place_location.position.y = y

        # move to above target square location
        success &= self.move_gripper(place_location)
        # move down to pick/place height
        place_location.position.z = self.height_setting["pick"]
        success &= self.move_gripper(place_location)
        # open the gripper
        success &= self.engine.reach_gripper_position(
            self.gripper_setting["open"])
        # move to above target square location
        place_location.position.z = self.height_setting["over"]
        success &= self.move_gripper(place_location)

        if success:
            print("placed piece successfully!")
        return success

    def trash_piece(self):
        success = True

        # set pose to graveyard position
        x, y, z = self.board.get_graveyard_coordinates()
        grave_location = self.engine.set_pose_msg(pos_x=x,
                                                  pos_y=y,
                                                  pos_z=z,
                                                  ori_w=self.gripper_ori["ori_w"],
                                                  ori_x=self.gripper_ori["ori_x"],
                                                  ori_y=self.gripper_ori["ori_y"],
                                                  ori_z=self.gripper_ori["ori_z"])
        # move to grave position
        success &= self.move_gripper(grave_location)

        # open gripper
        success &= self.engine.reach_gripper_position(
            self.gripper_setting["open"])
        if success:
            print("Piece trashed successfully!")
        return success

    def aruco_positions_callback(self, msg):
        if not self.aruco_positions_received:
            self.aruco_positions_received = True

        self.aruco_positions = msg.aruco_positions
        self.chess_board.reset_board()

        self.chess_board.populate_board(self.aruco_positions)

    def initial_calibration(self):
        while not self.aruco_positions_received:
            rospy.loginfo("Waiting for ArUco positions...")
            rospy.sleep(1)

        self.chess_board.calculate_global_orientation(self.aruco_positions)
        self.populate_board_position_map()
        self.chess_board.populate_board(self.aruco_positions)


if __name__ == "__main__":
    rospy.init_node('kinova_chess_control_node')
    kinova_chess_control = KinovaChessControl()

    # Wait for 'get_move' and 'capture_and_process_image' services to become available
    rospy.loginfo("Waiting for 'get_move' service...")
    rospy.wait_for_service('get_move')
    rospy.loginfo("'get_move' service is available.")

    rospy.loginfo("Waiting for 'capture_and_process_image' service...")
    rospy.wait_for_service('capture_and_process_image')
    rospy.loginfo("'capture_and_process_image' service is available.")

    get_move_client = rospy.ServiceProxy('get_move', GetMove)
    capture_and_process_image_client = rospy.ServiceProxy(
        'capture_and_process_image', CaptureAndProcessImage)

    kinova_chess_control.initial_calibration()

    while not rospy.is_shutdown():
        try:
            # Take and Process image and update aruco_positions
            # TODO Move and wait till arm is in position for photo
            req = CaptureAndProcessImageRequest()
            capture_and_process_image_client(req)
            input_prompt = "Input piece current position and desired position \n(e.g., c2c3 to move piece on c2 to c3) or type 'exit' to quit: "
            move_resp = get_move_client(input_prompt)
            if move_resp.current_pos == "" and move_resp.final_pos == "":
                continue
            kinova_chess_control.execute_chess_move(
                current_pos=move_resp.current_pos, final_pos=move_resp.final_pos, kill_move=move_resp.kill_move)
        except ValueError as e:
            print(e)
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
        except KeyboardInterrupt:
            break

    # Sanitize input
        # success = true
        # success &= len(input) == 4 -- check for 4 charectors
        # success &= -- make sure first is letter a b c d e f g h
        # success &= -- make sure second is number 1-8
        # success &= -- make sure third is letter a b c d e f g h
        # success &= -- make sure second is number 1-8
        # if ~success: print(input invalid)

    # Verify move
    # check if the desired piece is on the board.
    # verify_move(self, )
    # ChessBoard.Status("c2") -- is one of my pieces there?
    # if my piece is there continue
    # elseif no piece is there print(There's no piece there)
    # elseif opponents piece is there print(That's not my piece)

    # Check if there is a friendly at the desired position == bad move
    # check if there is a enemy at the position == run kill move
    # if there is no piece there  == run normal move
