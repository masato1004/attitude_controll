import numpy as np

class AttitudeController:
    def __init__(self, plate_length=0.05, plate_width=0.05, plate_thickness=0.005):
        self.PLATE_LENGTH = plate_length
        self.PLATE_WIDTH = plate_width
        self.PLATE_THICKNESS = plate_thickness
        
        self.pillar_lf = np.array([0.015, -0.015, 0])  # left front
        self.pillar_rf = np.array([0.015,  0.015, 0])  # right front
        self.pillar_cr = np.array([-0.015, 0, 0])      # center rear
        
        self.start_time = None
        # self.rolls = []
        # self.pitches = []
        # self.times = []
    
    def calculate_plate_planes(self, roll, pitch):
        half_length = self.PLATE_LENGTH / 2.0
        half_width = self.PLATE_WIDTH / 2.0
        half_thickness = self.PLATE_THICKNESS / 2.0

        # 8 corner combinations: (x, y, z) for x in +/-half_length, y in +/-half_width, z in +/-half_thickness
        corners = np.array([
            [-half_length, -half_width, -half_thickness],
            [ half_length, -half_width, -half_thickness],
            [ half_length,  half_width, -half_thickness],
            [-half_length,  half_width, -half_thickness],
            [-half_length, -half_width,  half_thickness],
            [ half_length, -half_width,  half_thickness],
            [ half_length,  half_width,  half_thickness],
            [-half_length,  half_width,  half_thickness],
        ], dtype=float)

        # Rotation matrices: roll about x-axis, pitch about y-axis
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)

        R_x = np.array([[1, 0, 0],
                        [0, cr, -sr],
                        [0, sr,  cr]])

        R_y = np.array([[ cp, 0, sp],
                        [  0, 1,  0],
                        [-sp, 0, cp]])

        # Apply roll first, then pitch: R = R_y @ R_x
        R = R_y @ R_x

        # Rotate corners (row vectors), so multiply by R.T
        rotated_corners = corners @ R.T

        # 6 faces (each face given by 4 corner indices)
        face_indices = [
            [0, 1, 2, 3],  # bottom face (z = -half_thickness)
            [4, 5, 6, 7],  # top face (z = +half_thickness)
            [0, 1, 5, 4],  # front face (y = -half_width)
            [2, 3, 7, 6],  # back face (y = +half_width)
            [1, 2, 6, 5],  # right face (x = +half_length)
            [0, 3, 7, 4],  # left face (x = -half_length)
        ]
        
        # planes: list of 4x3 arrays (points for each of the 6 planes)
        planes = [rotated_corners[np.array(idx)] for idx in face_indices]
        return planes
    
    def calculate_pillars_point(self, roll, pitch):
        # Rotation matrices: roll about x-axis, pitch about y-axis
        cr, sr = np.cos(-roll), np.sin(-roll)
        cp, sp = np.cos(pitch), np.sin(pitch)

        R_x = np.array([[1, 0, 0],
                        [0, cr, -sr],
                        [0, sr,  cr]])

        R_y = np.array([[ cp, 0, sp],
                        [  0, 1,  0],
                        [-sp, 0, cp]])

        # Apply roll first, then pitch: R = R_y @ R_x
        R = R_y @ R_x

        # Rotate pillar points (row vectors), so multiply by R.T
        pillar_lf_rotated = self.pillar_lf @ R.T
        pillar_rf_rotated = self.pillar_rf @ R.T
        pillar_cr_rotated = self.pillar_cr @ R.T

        return pillar_lf_rotated, pillar_rf_rotated, pillar_cr_rotated