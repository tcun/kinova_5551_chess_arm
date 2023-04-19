import numpy as np
import math


class ChessBoard:
    def __init__(self, corner_tags_coords):
        self.board = np.zeros((8, 8), dtype=object)
        # TODO Create board map that maps indexes to calculated cartesian coordinates
        self.board_position_map = np.zeros((8, 8), dtype=tuple)
        self.global_origin, self.global_scale = self.calculate_global_orientation(
            corner_tags_coords)
        self.graveyard_coordinates = [0, 0, 0]

    def calculate_global_orientation(self, aruco_positions):
        aruco_ids = [101, 102, 103, 104]
        marker_positions = {aruco_id: None for aruco_id in aruco_ids}

        for position in aruco_positions:
            if position.id in aruco_ids:
                marker_positions[position.id] = np.array(
                    [position.x, position.y, position.z])

        # Calculation for center of board
        normal_101_102 = marker_positions[102] - marker_positions[101]
        normal_103_104 = marker_positions[104] - marker_positions[103]
        normal_101_103 = marker_positions[103] - marker_positions[101]
        normal_102_104 = marker_positions[104] - marker_positions[102]

        center_1 = (normal_101_102 + normal_103_104) / 2
        center_2 = (normal_101_103 + normal_102_104) / 2
        center = (center_1 + center_2) / 2

        self.global_orientation = center

    def get_square_coordinates(self, position):
        # x_coordinate = (position[0] * self.global_scale[0]
        #                 ) + self.global_origin[0]
        # y_coordinate = (position[1] * self.global_scale[1]
        #                 ) + self.global_origin[1]
        theta = math.pi
        d_x = .2
        d_y = -.2

        # square size
        sq_inc = 0.035

        # board_number_map = {"1": 0.5*sq_inc, "2": 1.5*sq_inc, "3": 2.5*sq_inc,
        #                     "4": 3.5*sq_inc, "5": 4.5*sq_inc, "6": 5.5*sq_inc,
        #                     "7": 6.5*sq_inc, "8": 7.5*sq_inc}
        
        board_number_map = np.array(
            [0.5*sq_inc, 1.5*sq_inc, 2.5*sq_inc, 3.5*sq_inc, 4.5*sq_inc, 5.5*sq_inc, 6.5*sq_inc, 7.5*sq_inc])

        # vector maps letter to x coordinate and number to y coordinate
        square_vector = np.array([board_number_map[position[0]],
                                  board_number_map[position[1]],
                                  0])

        # rotations will always be around the z axis
        R_wb = np.array([[math.cos(theta), -math.sin(theta), 0],
                        [math.sin(theta), math.cos(theta), 0],
                        [0, 0, 1]])

        world_coord = np.matmul(R_wb, np.transpose(square_vector))

        x_coordinate = world_coord[0] + d_x
        y_coordinate = world_coord[1] + d_y

        return x_coordinate, y_coordinate
    
    def populate_board_position_map(self):
        for i in range(8):
            for j in range(8):
                x, y = self.get_square_coordinates([i,j])
                self.board_position_map[i, j] = (x, y)

    def get_square_status(self, position):
        piece = self.board[position[0], position[1]]
        if piece:
            return piece
        return None

    def set_square_status(self, position, piece):
        self.board[position[0], position[1]] = piece

    def get_graveyard_coordinates(self):

        return self.graveyard_coordinates

    def set_graveyard_coordinates(self, x, y, z):
        self.graveyard_coordinates = [x, y, z]

    def populate_board(self, aruco_positions):
        for fiducial_id, x, y, z in aruco_positions:  # ???
            if 201 <= fiducial_id <= 316:
                team_type, piece_type = self.get_piece_type_and_color(
                    fiducial_id)
                cartesian_position = (x, y, z)
                board_position = self.get_board_position(cartesian_position)
                piece = ChessPiece(
                    team_type, piece_type, fiducial_id, cartesian_position, board_position)
                self.add_piece_to_board(piece)
            else:
                # Add ROS error log
                pass

    def get_piece_type_and_color(self, fiducial_id):
        team_type = None
        piece_type = None

        if 201 <= fiducial_id <= 208:
            team_type = 'black'
            piece_type = 'pawn'
        elif fiducial_id in [209, 210]:
            team_type = 'black'
            piece_type = 'rook'
        elif fiducial_id in [211, 212]:
            team_type = 'black'
            piece_type = 'knight'
        elif fiducial_id in [213, 214]:
            team_type = 'black'
            piece_type = 'bishop'
        elif fiducial_id == 215:
            team_type = 'black'
            piece_type = 'queen'
        elif fiducial_id == 216:
            team_type = 'black'
            piece_type = 'king'
        elif 301 <= fiducial_id <= 308:
            team_type = 'white'
            piece_type = 'pawn'
        elif fiducial_id in [309, 310]:
            team_type = 'white'
            piece_type = 'rook'
        elif fiducial_id in [311, 312]:
            team_type = 'white'
            piece_type = 'knight'
        elif fiducial_id in [313, 314]:
            team_type = 'white'
            piece_type = 'bishop'
        elif fiducial_id == 315:
            team_type = 'white'
            piece_type = 'queen'
        elif fiducial_id == 316:
            team_type = 'white'
            piece_type = 'king'
        else:
            pass

        return team_type, piece_type

    def get_board_position(self, cartesian_position):
        min_distance = float('inf')
        nearest_position = None

        for i in range(8):
            for j in range(8):
                board_x, board_y = self.board_position_map[i, j]
                distance = np.linalg.norm(np.array([board_x, board_y]) - np.array(cartesian_position[:2]))

                if distance < min_distance:
                    min_distance = distance
                    nearest_position = self.index_to_position(i, j)

        return nearest_position

    def add_piece_to_board(self, piece):
        row, col = piece.board_position
        self.board[row, col] = piece

    def reset_board(self):
        self.board = np.zeros((8, 8), dtype=object)


class ChessPiece:
    def __init__(self, team_type, piece_type, aruco_id, cartesian_position, board_position):
        self.team_type = team_type
        self.piece_type = piece_type
        self.aruco_id = aruco_id
        self.cartesian_position = cartesian_position
        self.board_position = board_position

    def is_friendly(self, piece_type):
        return self.piece_type == piece_type

    def get_world_coord(self):
        return self.position

    def set_world_coord(self, x, y):
        self.position = [x, y]

    def __str__(self):
        return f"{self.piece_type} ({self.team_type}) at {self.position}"
