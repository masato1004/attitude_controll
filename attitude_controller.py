import numpy as np
import time

class AttitudeController:
    CENTROID_DISTANCE = 0.02  # distance from the vertex to the centroid
    ARM_LENGTH = 0.017
    def __init__(self, plate_length=0.04, plate_width=0.04, plate_thickness=0.01, control_cycle=0.02):
        self.PLATE_LENGTH = plate_length
        self.PLATE_WIDTH = plate_width
        self.PLATE_THICKNESS = plate_thickness
        
        self._status = True
        self._control_cycle = control_cycle
        
        self.pillar_lr = np.array([-AttitudeController.CENTROID_DISTANCE*np.sin(np.pi/6), -AttitudeController.CENTROID_DISTANCE*np.cos(np.pi/6), 0])
        self.pillar_rr = np.array([-AttitudeController.CENTROID_DISTANCE*np.sin(np.pi/6), AttitudeController.CENTROID_DISTANCE*np.cos(np.pi/6), 0])
        self.pillar_cf = np.array([AttitudeController.CENTROID_DISTANCE, 0, 0])
        
        self.servo_agl_lr = 0
        self.servo_agl_rr = 0
        self.servo_agl_cf = 0
    
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

        # Rotate pillar points (row vectors), so multiply by R.T
        pillar_lr_rotated = self.pillar_lr @ R.T
        pillar_rr_rotated = self.pillar_rr @ R.T
        pillar_cf_rotated = self.pillar_cf @ R.T

        return pillar_lr_rotated, pillar_rr_rotated, pillar_cf_rotated
    
    def calculate_arm_angle(self, roll:float, pitch:float) -> list:
        '''
        Docstring of calculate_arm_angle
        
        :param roll: [rad]
        :param pitch: [rad]
        
        ---
        
        :Output Description:
        This returns backward calculated each servo angle [deg]
        '''
        pillar_lr_rotated, pillar_rr_rotated, pillar_cf_rotated = self.calculate_pillars_point(roll, pitch)
        
        self.servo_agl_lr = np.arcsin(pillar_lr_rotated[2] / AttitudeController.ARM_LENGTH)
        self.servo_agl_rr = np.arcsin(pillar_rr_rotated[2] / AttitudeController.ARM_LENGTH)
        self.servo_agl_cf = np.arcsin(pillar_cf_rotated[2] / AttitudeController.ARM_LENGTH)
        
        # print(f"{np.rad2deg(self.servo_agl_lr):.2f}, {np.rad2deg(self.servo_agl_rr):.2f}, {np.rad2deg(self.servo_agl_cf):.2f}")
        # return self.servo_agl_lr, self.servo_agl_rr, self.servo_agl_cf
    
    @property
    def target_angle_differential(self):
        return self.servo_agl_lr, self.servo_agl_rr, self.servo_agl_cf
    
    def loop(self, reader):
        while self._status:
            time.sleep(self._control_cycle)
            self.calculate_arm_angle(reader.roll, reader.pitch)
    
    def stop_controller(self):
        self._status = False