import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import time

from stl import mesh
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
        # self.line_roll, = self.ax.plot([], [], label=f'Roll (degrees) {self.name}')
        # self.line_pitch, = self.ax.plot([], [], label=f'Pitch (degrees) {self.name}')
        # self.ax.set_ylabel(f'Angle by {self.name} (degrees)')
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
        # print(np.abs(np.rad2deg(roll)))
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
    
    def draw_seat(self, roll, pitch):
        # create a simple concave seat + backrest mesh if not already created
        if not hasattr(self, 'seat_mesh_base') or self.seat_mesh_base is None:
            L = 0.020         # seat length (m)
            W = 0.020         # seat width (m)
            T = 0.003         # seat thickness (m)
            BACK_H = 0.04    # backrest height (m)
            nx, ny = 28, 28  # grid resolution for the seat surface

            xs = np.linspace(-L/2, L/2, nx)
            ys = np.linspace(-W/2, W/2, ny)
            X, Y = np.meshgrid(xs, ys, indexing='ij')

            # concave bowl shape for the seat surface
            Z_top = -0.02 * np.exp(-((2*X/L)**2 + (2*Y/W)**2))

            top_pts = np.stack([X, Y, Z_top], axis=-1)         # (nx, ny, 3)
            bottom_pts = top_pts.copy()
            bottom_pts[..., 2] -= T

            tris = []
            def add_tri(a, b, c):
                tris.append([a, b, c])

            # top surface triangles
            for i in range(nx-1):
                for j in range(ny-1):
                    p00 = top_pts[i, j]
                    p10 = top_pts[i+1, j]
                    p11 = top_pts[i+1, j+1]
                    p01 = top_pts[i, j+1]
                    add_tri(p00, p10, p11)
                    add_tri(p00, p11, p01)

            # bottom surface triangles (reversed winding)
            for i in range(nx-1):
                for j in range(ny-1):
                    p00 = bottom_pts[i, j]
                    p10 = bottom_pts[i+1, j]
                    p11 = bottom_pts[i+1, j+1]
                    p01 = bottom_pts[i, j+1]
                    add_tri(p11, p10, p00)
                    add_tri(p01, p11, p00)

            # side faces connecting top and bottom (four edges)
            # front (j=0) and back (j=ny-1)
            for i in range(nx-1):
                # front
                t0 = top_pts[i, 0]; t1 = top_pts[i+1, 0]
                b0 = bottom_pts[i, 0]; b1 = bottom_pts[i+1, 0]
                add_tri(t0, t1, b1); add_tri(t0, b1, b0)
                # back
                t0 = top_pts[i, ny-1]; t1 = top_pts[i+1, ny-1]
                b0 = bottom_pts[i, ny-1]; b1 = bottom_pts[i+1, ny-1]
                add_tri(t1, t0, b1); add_tri(t0, b0, b1)

            # left (i=0) and right (i=nx-1)
            for j in range(ny-1):
                # left
                t0 = top_pts[0, j]; t1 = top_pts[0, j+1]
                b0 = bottom_pts[0, j]; b1 = bottom_pts[0, j+1]
                add_tri(t1, t0, b1); add_tri(t0, b0, b1)
                # right
                t0 = top_pts[nx-1, j]; t1 = top_pts[nx-1, j+1]
                b0 = bottom_pts[nx-1, j]; b1 = bottom_pts[nx-1, j+1]
                add_tri(t0, t1, b1); add_tri(t0, b1, b0)

            # simple backrest attached to the back edge (y = +W/2)
            for i in range(nx-1):
                p_top = top_pts[i, ny-1]
                p_top2 = top_pts[i+1, ny-1]
                p_back_top = p_top.copy(); p_back_top[2] += BACK_H
                p_back_top2 = p_top2.copy(); p_back_top2[2] += BACK_H
                add_tri(p_top, p_top2, p_back_top2)
                add_tri(p_top, p_back_top2, p_back_top)

            vectors = np.array(tris, dtype=float)
            seat_mesh_base = mesh.Mesh(np.zeros(vectors.shape[0], dtype=mesh.Mesh.dtype))
            seat_mesh_base.vectors = vectors
            self.seat_mesh_base = seat_mesh_base

        # remove previous seat collection from axes if present
        if hasattr(self, 'seat_collection'):
            try:
                if self.seat_collection in self.ax.collections:
                    self.ax.collections.remove(self.seat_collection)
            except Exception:
                pass

        # Rotate seat mesh to match current roll/pitch (uses global roll, pitch if available)
        ang_roll = roll
        ang_pitch= pitch

        # Rotation matrices: roll about x-axis, pitch about y-axis
        cr, sr = np.cos(ang_roll), np.sin(ang_roll)
        cp, sp = np.cos(ang_pitch), np.sin(ang_pitch)

        R_x = np.array([[1, 0, 0],
                        [0, cr, -sr],
                        [0, sr,  cr]])

        R_y = np.array([[ cp, 0, sp],
                        [  0, 1,  0],
                        [-sp, 0, cp]])

        R = R_y @ R_x

        # Apply rotation about the mesh centroid to avoid unwanted translation
        vectors = self.seat_mesh_base.vectors.copy()  # shape: (n_facets, 3, 3)
        pts = vectors.reshape(-1, 3)
        centroid = pts.mean(axis=0)

        pts_centered = pts - centroid
        pts_rotated = pts_centered @ R.T
        pts_final = pts_rotated + centroid

        # Update a copy of the mesh vectors so original file can be reloaded if needed
        self.seat_mesh = mesh.Mesh(np.zeros(vectors.shape[0], dtype=mesh.Mesh.dtype))
        self.seat_mesh.vectors = pts_final.reshape(vectors.shape)
        # Create and add updated seat mesh collection
        self.seat_collection = Poly3DCollection(self.seat_mesh.vectors, facecolors=[0.5, 0.5, 1], linewidths=1, edgecolors='k', alpha=0.25)
        self.ax.add_collection3d(self.seat_collection)

        # Redraw canvas
        self.fig.canvas.draw_idle()
        self.fig.canvas.draw()
        plt.pause(0.03)
        
    
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
    
    # def read():
    #     global rt
    #     global roll, pitch
        
    #     while True:
    #         rx = com.recv()  # format: roll_pitch
    #         if rx != "":
    #             index = rx.rfind('s')
    #             if index != -1:
    #                 rx = rx[index+1:]
    #                 # print(rx)
    #                 roll, pitch = rx.split('_')
    #                 roll, pitch = float(roll), float(pitch)
    
    file_path = "seat_low_res.stl"
    seat = mesh.Mesh.from_file(file_path)

    com = tc(ip="192.168.1.228", port=50001, hosting=True, data_num=2, seperator="_")
    com.connect()
    
    att_ctrl = AttCtrl()
    
    rtv = RealTimeViewer()
    time.sleep(1)
    roll, pitch = 0, 0
    
    # q = Queue()
    
    # thread = Thread(target=read)
    thread = Thread(target=com.loop_recv)
    thread.setDaemon(True)
    thread.start()
    
    while True:
        try:
            
            # print(f"Roll: {float(roll):.2f}, Pitch: {float(pitch):.2f}")
            data = com.get_data(starts_with='s')
            roll_str, pitch_str = data.split("_")
            roll, pitch = float(roll_str), float(pitch_str)
            
            controll_points = att_ctrl.calculate_pillars_point(roll, pitch)
            rtv.plot_point(controll_points)
            rtv.add_data(roll, pitch, time.time())
            # rtv.draw_seat(roll, pitch)
            
            # rtv.add_data_2d(roll, pitch, time.time())
            
        except KeyboardInterrupt:
            com.close()
            rtv.close()
            break