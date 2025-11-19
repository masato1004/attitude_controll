import matplotlib.pyplot as plt
import numpy as np

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
    def __init__(self, name='Real-Time Angles'):
        self.rolls = []
        self.pitches = []
        self.times = []
        self.start_time = None
        self.name = name
        plt.ion()  # Turn on interactive mode
        self.fig, self.ax = plt.subplots()
        self.line_roll, = self.ax.plot([], [], label=f'Roll (degrees) {self.name}')
        self.line_pitch, = self.ax.plot([], [], label=f'Pitch (degrees) {self.name}')
        self.ax.set_xlabel('Time (s)')
        self.ax.set_ylabel(f'Angle by {self.name} (degrees)')
        self.ax.set_title('Real-Time Roll and Pitch Angles')
        self.ax.legend()
        self.ax.grid(True)

    def add_data(self, roll, pitch, current_time):
        if self.start_time is None:
            self.start_time = current_time
        self.rolls.append(roll)
        self.pitches.append(pitch)
        self.times.append(current_time - self.start_time)
        self.update_plot()

    def update_plot(self):
        self.line_roll.set_data(self.times, np.degrees(self.rolls))
        self.line_pitch.set_data(self.times, np.degrees(self.pitches))
        self.ax.relim()
        self.ax.autoscale_view()
        plt.draw()
        # plt.pause(0.01)