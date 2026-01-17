import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import time
from queue import Queue
from threading import Thread

from communicator import TcpCommunicator as tc
from attitude_controller import AttitudeController as AttCtrl

class AngleViewer:
    def __init__(self, name='Angles'):
        self.rolls = []
        self.pitches = []
        self.times = []
        self.start_time = None
        self.name = name

    def add_data(self, roll, pitch, current_time):
        if self.start_time is None:
            self.start_time = current_time
        self.rolls.append(roll)
        self.pitches.append(pitch)
        self.times.append(current_time - self.start_time)

    def plot(self):
        # plt.figure(figsize=(10, 5))
        plt.plot(self.times, np.degrees(self.rolls), label=f'Roll (degrees) {self.name}') #, color='r'
        plt.plot(self.times, np.degrees(self.pitches), label=f'Pitch (degrees) {self.name}') #, color='b'
        plt.xlabel('Time (s)')
        plt.ylabel(f'Angle by {self.name} (degrees)')
        plt.title('Roll and Pitch Angles Over Time')
        plt.legend()
        plt.grid(True)
        plt.show()

class RealTimeViewer:
    PLATE_LENGTH = 0.04
    PLATE_WIDTH = 0.04
    PLATE_THICKNESS = 0.01
    def __init__(self, name='Real-Time Angles'):
        self.rolls = []
        self.pitches = []
        self.times = []
        self.start_time = None
        self.name = name
        plt.ion()  # Turn on interactive mode
        # self.fig, self.ax = plt.subplots()
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        # self.ax.add_collection3d(Poly3DCollection(np.zeros((4, 3)), facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.5))
        self.ax.add_collection3d(Poly3DCollection([], facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.5))
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.line_roll, = self.ax.plot([], [], label=f'Roll (degrees) {self.name}')
        self.line_pitch, = self.ax.plot([], [], label=f'Pitch (degrees) {self.name}')
        self.ax.set_ylabel(f'Angle by {self.name} (degrees)')
        self.ax.set_title('Real-Time Roll and Pitch Angles')
        self.ax.legend(loc='upper right')
        self.ax.grid(True, color='gray', linestyle='--', linewidth=0.5)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        # self.set_needs_display()
    
    def calculate_plate_corners(self, roll, pitch):
        """
        Return the 8 corners of a rectangular plate (length x width x thickness)
        centered at the origin and rotated by roll (about the x-axis) and
        pitch (about the y-axis). Angles are expected in radians.
        """
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
    
    def add_data(self, roll, pitch, current_time, update=True):
        if self.start_time is None:
            self.start_time = current_time
        # self.rolls.append(roll)
        # self.pitches.append(pitch)
        # self.times.append(current_time - self.start_time)
        if update:
            self.update_plot(roll, pitch)
            
    def update_plot(self, roll, pitch):
        # Calculate new plate corners
        planes = self.calculate_plate_corners(roll, pitch)
        
        # Clear previous plate
        while len(self.ax.collections) > 0:
            self.ax.collections[0].remove()
        
        
        # Create Poly3DCollection for the plate
        plate = None
        if np.abs(roll) > np.deg2rad(11) or np.abs(pitch) > np.deg2rad(11):
            plate = Poly3DCollection(planes, facecolors='red', linewidths=1, edgecolors='r', alpha=0.2)
        else:
            plate = Poly3DCollection(planes, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.5)
        self.ax.add_collection3d(plate)
        
        for point in self.controll_points:
            point = np.array(point).reshape(3,-1).flatten()
            # print("Height error (m):", point[2])
            self.ax.scatter([point[0]], [point[1]], [point[2]], color='pink', s=40)
            # self.ax.added3D(art3d.Line3DCollection([point.reshape(3,1).T], colors='pink', linewidths=5))
        
        # Set limits
        limit = 0.07
        self.ax.set_xlim([-limit, limit])
        self.ax.set_ylim([-limit, limit])
        self.ax.set_zlim([-limit, limit])
        
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        # self.fig.canvas.flush_events()
        # self.set_needs_display()
        plt.pause(0.03)
    
    def plot_point(self, points):
        self.controll_points = points
    
    def close(self):
        plt.ioff()
        plt.close(self.fig)

    def add_data_2d(self, roll, pitch, current_time):
        if self.start_time is None:
            self.start_time = current_time
        self.rolls.append(roll)
        self.pitches.append(pitch)
        self.times.append(current_time - self.start_time)
        self.update_plot_2d()

    def update_plot_2d(self):
        self.line_roll.set_data(self.times, np.degrees(self.rolls))
        self.line_pitch.set_data(self.times, np.degrees(self.pitches))
        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.01)
        self.fig.canvas.draw_idle()  # Non-blocking draw
        
if __name__ == "__main__":
    
    def read():
        global rt
        global roll, pitch
        
        while True:
            rx = com.recv()  # format: roll_pitch
            if rx != "":
                index = rx.rfind('s')
                if index != -1:
                    rx = rx[index+1:]
                    # print(rx)
                    roll, pitch = rx.split('_')
                    roll, pitch = float(roll), float(pitch)
        
    com = tc(ip="192.168.1.228", port=50001, hosting=True)
    com.connect()
    
    att_ctrl = AttCtrl()
    
    rt = RealTimeViewer()
    time.sleep(1)
    roll, pitch = 0, 0
    
    # q = Queue()
    
    thread = Thread(target=read)
    thread.setDaemon(True)
    thread.start()
    
    while True:
        try:
            
            # print(f"Roll: {float(roll):.2f}, Pitch: {float(pitch):.2f}")
            
            controll_points = att_ctrl.calculate_pillars_point(roll, pitch)
            rt.plot_point(controll_points)
            rt.add_data(roll, pitch, time.time())
            att_ctrl.calculate_arm_angle(roll, pitch)
            
            # rt.add_data_2d(roll, pitch, time.time())
            
        except KeyboardInterrupt:
            com.close()
            rt.close()
            break