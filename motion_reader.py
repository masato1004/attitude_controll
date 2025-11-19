import motion
from time import sleep
import numpy as np

class MotionReader():
    def __init__(self, num_samples=500, sleep_time=0.005):
        self.num_samples = num_samples
        self.sleep_time = sleep_time
        self._acc_list = []
        self._gyro_list = []
        self._mag_list = []
        self._acc_x, self._acc_y, self._acc_z = 0, 0, 0
        self._gyro_roll, self._gyro_pitch, self._gyro_yaw = 0, 0, 0
        self._mag_x, self._mag_y, self._mag_z, self._mag_accuracy = 0, 0, 0, 0
        self._roll_by_acc, self._pitch_by_acc = 0, 0
        self._status = False
    
    def start(self):
        motion.start_updates()
        sleep(0.2)
        self._status = True
    
    def stop(self):
        motion.stop_updates()
        self._status = False
    
    def read_acceleration(self):
        # acc = motion.get_user_acceleration()
        acc = motion.get_gravity()
        self._acc_x, self._acc_y, self._acc_z = acc
        self._acc_list.append(acc)
        if (len(self._acc_list) > self.num_samples):
            self._acc_list.pop(0)
            
    def read_gyro(self):
        gyro = motion.get_attitude()
        self._gyro_roll, self._gyro_pitch, self._gyro_yaw = gyro
        self._gyro_list.append(gyro)
        if (len(self._gyro_list) > self.num_samples):
            self._gyro_list.pop(0)
            
    def read_magnetic_field(self):  # Fixed typo in method name
        mag = motion.get_magnetic_field()
        self._mag_x, self._mag_y, self._mag_z, self._mag_accuracy = mag
        self._mag_list.append(mag)
        if (len(self._mag_list) > self.num_samples):
            self._mag_list.pop(0)
        
    def acc2euler(self, ax, ay, az):
        roll = np.arctan2(-ay, -az)
        pitch = np.arctan2(ax, np.sqrt(ay * ay + az * az))
        return roll, pitch
    
    def loop(self):
        if not self._status:
            raise RuntimeError("MotionReader is not started. Call start() before loop().")
        while self._status:
            self.read_acceleration()
            self.read_gyro()
            self.read_magnetic_field()
            sleep(self.sleep_time)
        
    @property
    def acc(self):
        return self._acc_x, self._acc_y, self._acc_z
    
    @property
    def gyro(self):
        return self._gyro_roll, self._gyro_pitch, self._gyro_yaw
    
    @property
    def mag(self):
        return self._mag_x, self._mag_y, self._mag_z
    
    @property
    def omega(self):
        diff_gyro = np.diff(self._gyro_list, axis=0)
        if len(diff_gyro) == 0:
            return 0.0, 0.0, 0.0
        omega = diff_gyro / self.sleep_time
        return omega[-1]  # Return the latest angular velocity
