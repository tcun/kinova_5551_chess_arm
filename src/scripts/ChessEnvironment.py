import numpy as np
import math

class ChessBoard:
    def __init__(self):
        # Initialize empty chess board
        self.board = np.zeros((8, 8), dtype=object)
        
        # Initialize board position map that maps indexes to calculated cartesian coordinates
        self.board_position_map = np.zeros((8, 8), dtype=tuple)
        
        # Initialize global origin and scale for board coordinates
        self.global_origin, self.global_scale = (0, 0)
        
        # Initialize coordinates for graveyard for captured pieces
        self.graveyard_coordinates = [0, 0, 0]

    def calculate_global_orientation(self, aruco_positions):
        # Calculate global orientation of chess board based on Aruco marker positions
        aruco_ids = [101, 102]
        marker_positions = {aruco_id: None for aruco_id in aruco_ids}

        for position in aruco_positions:
            if position.id in aruco_ids:
                marker_positions[position.id] = np.array([position.x, position.y, position.z])

        # Calculation for center of board
        normal_101_102 = marker_positions[102] - marker_positions[101]
        
        # Calculate angle between y_world and y_board
        theta = math.atan2(normal_101_102[0], normal_101_102[1])

        # Set origin coordinates for chess board
        board_x = marker_positions[101][0]
        board_y = marker_positions[101][1]

        # Set global orientation for chess board
        self.global_orientation = board_x, board_y, theta

    def get_square_coordinates(self, position):
        # Calculate world coordinates of square based on board position and global orientation
        d_x, d_y, theta = self.global_orientation

        # Set square size
        sq_inc = 0.035

        # Create map for board number to position offset
        board_number_map = np.array([0.5*sq_inc, 1.5*sq_inc, 2.5*sq_inc, 3.5*sq_inc,
                                     4.5*sq_inc, 5.5*sq_inc, 6.5*sq_inc, 7.5*sq_inc])

        # Map letter and number positions to position offsets
        square_vector = np.array([board_number_map[position[0]], board_number_map[position[1]], 0])

        # Apply rotation matrix to calculate world coordinates
        R_wb = np.array([[math.cos(theta), -math.sin(theta), 0],
                         [math.sin(theta), math.cos(theta), 0],
                         [0, 0, 1]])
        world_coord = np.matmul(R_wb, np.transpose(square_vector))

        # Calculate final world coordinates
        x_coordinate = world_coord[0] + d_x
        y_coordinate = world_coord[1] + d_y

        return x_coordinate, y_coordinate

    def populate_board_position_map(self):
        # Populate board position map with world coordinates of each square
        for i in range(8):
            for j in range(8):
                x, y = self.get_square_coordinates([i,j])
                self.board_position_map[i, j] = (x, y)

    def get_square_status(self, position):
        # Return piece on board at given position, if any
        piece = self.board[position[0], position[1]]
        if piece:
            return piece
        return None

    def set_square_status(self, position, piece):
        # Set piece on board at given position
        self.board[position[0], position[1]] = piece

    def get_graveyard_coordinates(self):
        # Return world coordinates of graveyard for captured pieces
        return self.graveyard_coordinates

    def set_graveyard_coordinates(self, x, y, z):
        # Set world coordinates of graveyard for captured pieces
        self.graveyard_coordinates = [x, y, z]

    def populate_board(self, aruco_positions):
        # Populate chess board with pieces based on Aruco marker positions
        for fiducial_id, x, y, z in aruco_positions:
            if 201 <= fiducial_id <= 316:
                team_type, piece_type = self.get_piece_type_and_color(fiducial_id)
                cartesian_position = (x, y, z)
                board_position = self.get_board_position(cartesian_position)
                piece = ChessPiece(team_type, piece_type, fiducial_id, cartesian_position, board_position)
                self.add_piece_to_board(piece)
            else:
                # TODO: Add ROS error log
                pass

    def get_piece_type_and_color(self, fiducial_id):
        # Map fiducial IDs to piece types and colors
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
        # Determine board position based on cartesian position
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
        # Add piece to chess board
        row, col = piece.board_position
        self.board[row, col] = piece

    def reset_board(self):
        # Reset chess board to empty state
        self.board = np.zeros((8, 8), dtype=object)

class ChessPiece:
    def init(self, team_type, piece_type, aruco_id, cartesian_position, board_position):
        # Initialize ChessPiece with team type, piece type, Aruco ID, cartesian position, and board position
        self.team_type = team_type
        self.piece_type = piece_type
        self.aruco_id = aruco_id
        self.cartesian_position = cartesian_position
        self.board_position = board_position
        
    def is_friendly(self, piece_type):
        # Check if piece is friendly (i.e. same team)
        return self.piece_type == piece_type

    def get_world_coord(self):
        # Return cartesian position of piece
        return self.cartesian_position

    def set_world_coord(self, x, y):
        # Set cartesian position of piece
        self.cartesian_position = [x, y]

    def __str__(self):
        # Return string representation of ChessPiece
        return f"{self.piece_type} ({self.team_type}) at {self.cartesian_position}"

