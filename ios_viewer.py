import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import art3d
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.backends.backend_agg import FigureCanvasAgg
import numpy as np
from PIL import Image
import ui
import io
import time
from time import sleep

class RealTimeViewerIOS(ui.View):
    def __init__(self, name='Real-Time Angles', frame=(0, 0, 100, 90)):
        super().__init__(frame=frame)
        self.rolls = []
        self.pitches = []
        self.times = []
        self.start_time = None
        self.name = name
        plt.style.use('seaborn-darkgrid')
        self.fig, self.ax = plt.subplots(figsize=(4, 4), dpi=100)
        self.fig.patch.set_facecolor('white')
        self.ax.set_facecolor('white')
        self.line_roll, = self.ax.plot([], [], label=f'Roll (degrees) {self.name}')
        self.line_pitch, = self.ax.plot([], [], label=f'Pitch (degrees) {self.name}')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel(f'Angle by {self.name} (degrees)')
        self.ax.set_title('Real-Time Roll and Pitch Angles')
        self.ax.legend(loc='upper right')
        self.ax.grid(True, color='gray', linestyle='--', linewidth=0.5)
        # plt.show()
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.set_needs_display()
        # self.present('sheet')

    def add_data(self, roll, pitch, current_time, update=True):
        if self.start_time is None:
            self.start_time = current_time
        self.rolls.append(roll)
        self.pitches.append(pitch)
        self.times.append(current_time - self.start_time)
        if update:
            self.update_plot()

    def update_plot(self):
        self.line_roll.set_data(self.times, np.degrees(self.rolls))
        self.line_pitch.set_data(self.times, np.degrees(self.pitches))
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        # self.fig.canvas.flush_events()
        self.set_needs_display()
        # plt.pause(0.03)
    
    def draw(self):
        renderer = FigureCanvasAgg(self.fig)
        renderer.draw()
        w, h = self.fig.canvas.get_width_height()
        buf = renderer.buffer_rgba()
        img_pil = Image.frombuffer("RGBA", (w, h), bytes(buf), "raw", "RGBA", 0, 1)
        img_pil = img_pil.convert("RGB")
        with io.BytesIO() as output:
            img_pil.save(output, format="PNG")
            png_bytes = output.getvalue()
        img = ui.Image.from_data(png_bytes)
        if img != None:
            img.draw(0, 0, w, h)

class RealTime3DViewerIOS(ui.View):
    NAME = 'iPhone'
    PLATE_LENGTH = 0.05
    PLATE_WIDTH = 0.05
    PLATE_THICKNESS = 0.005
    def __init__(self, name='Real-Time 3D Angles', frame=(0, 0, 100, 90)):
    # def __init__(self, name='Real-Time 3D Angles', frame=(0, 0, 600, 600)):
        super().__init__(frame=frame)
        self.name = name
        self.rolls = []
        self.pitches = []
        self.controll_points = []
        self.times = []
        self.start_time = None
        self.name = name
        plt.style.use('seaborn-darkgrid')
        self.fig = plt.figure(figsize=(4, 4), dpi=100)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig.patch.set_facecolor('white')
        self.ax.set_facecolor('white')
        self.ax.add_collection3d(Poly3DCollection(np.array([[0,0,0]]*4), facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.5))
        self.ax.set_xlabel('X [m]')
        self.ax.set_ylabel('Y [m]')
        self.ax.set_zlabel('Z [m]')
        self.ax.set_title('Real-Time Roll and Pitch Angles')
        self.ax.legend(loc='upper right')
        self.ax.grid(True, color='gray', linestyle='--', linewidth=0.5)
        self.ax.view_init(elev=30, azim=0)
        # plt.show()
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()
        self.set_needs_display()
        # Additional initialization for 3D plotting can be added here
    
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
            self.update_plot(-roll, pitch)
            
    def update_plot(self, roll, pitch):
        # Calculate new plate corners
        planes = self.calculate_plate_corners(roll, pitch)
        
        # Clear previous plate
        self.ax.collections.clear()
        
        # Create Poly3DCollection for the plate
        plate = None
        if np.abs(roll) > np.deg2rad(11) or np.abs(pitch) > np.deg2rad(11):
            plate = Poly3DCollection(planes, facecolors='red', linewidths=1, edgecolors='r', alpha=0.5)
        else:
            plate = Poly3DCollection(planes, facecolors='cyan', linewidths=1, edgecolors='r', alpha=0.5)
        self.ax.add_collection3d(plate)
        
        for point in self.controll_points:
            point = np.array(point).reshape(3,-1).flatten()
            print("Height error (m):", point[2])
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
        self.set_needs_display()
        # plt.pause(0.03)
        
    def draw(self):
        renderer = FigureCanvasAgg(self.fig)
        renderer.draw()
        w, h = self.fig.canvas.get_width_height()
        buf = renderer.buffer_rgba()
        img_pil = Image.frombuffer("RGBA", (w, h), bytes(buf), "raw", "RGBA", 0, 1)
        img_pil = img_pil.convert("RGB")
        with io.BytesIO() as output:
            img_pil.save(output, format="PNG")
            png_bytes = output.getvalue()
        img = ui.Image.from_data(png_bytes)
        if img != None:
            img.draw(0, 0, w, h)
    
    def plot_point(self, points):
        self.controll_points = points
        # for point in points:
        #     point = np.array(point).reshape(3,-1).flatten()
        #     self.ax.scatter([point[0]], [point[1]], [point[2]], color='pink', s=50)
        # self.fig.canvas.draw()
        # self.set_needs_display()