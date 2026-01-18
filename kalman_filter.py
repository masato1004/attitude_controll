import numpy as np
from time import time, sleep

class EKF_RollPitch:
    def __init__(self, dt=0.01):
        self.dt = dt
        self.x = np.zeros((4,1))  # [roll, pitch, bx, by]
        self.P = np.eye(4)*0.1
        self.Q = np.eye(4)*0.01   # System noise
        self.R = np.eye(3)*0.05   # Observation noise
        self.status = False

    def predict(self, gyro):
        gx, gy = gyro
        roll, pitch, bx, by = self.x.flatten()

        # State prediction
        roll += (gx - bx) * self.dt
        pitch += (gy - by) * self.dt

        F = np.eye(4)
        self.x = np.array([[roll], [pitch], [bx], [by]])
        self.P = F @ self.P @ F.T + self.Q

    def update(self, acc):
        ax, ay, az = acc / np.linalg.norm(acc)  # Normalize acceleration vector
        roll, pitch, _, _ = self.x.flatten()

        # Predicted gravity direction
        g_pred = np.array([
            -np.sin(pitch),
            np.sin(roll)*np.cos(pitch),
            -np.cos(roll)*np.cos(pitch)
        ])

        # Observation residual
        y = np.array([[ax], [ay], [az]]) - g_pred.reshape(3,1)

        # Jacobian H (to be derived analytically or approximated numerically)
        H = np.zeros((3,4))  # Only roll/pitch-related parts are non-zero
        # ...expand H if needed...

        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
    
    def get_angles(self):
        roll, pitch, _, _ = self.x.flatten()
        return roll, pitch

class ExtendedKalmanFilter():
    def __init__(self, num_states: int = 2, num_measurements: int = 3):
        self.num_states = num_states # roll, pitch
        self.num_measurements = num_measurements
        self.num_measurements_omega = 3
        self.num_measurements_acc = 3
        self.status = False
        self.estimation_cycle = 0.01 # Added estimation_cycle attribute
        self.estimation_cycle_reality = 0.012
        self._estimated_roll = 0.0
        self._estimated_pitch = 0.0
        
        # State vector
        self.x = np.zeros((num_states, 1)).reshape(-1,1)
        
        # State covariance matrix
        self.P = np.ones((num_states,num_states)) * 3
        
        # Measurement matrix
        # self.H = np.hstack((np.eye(num_measurements), np.zeros((num_measurements, num_states - num_measurements))))
        
        # Measurement noise covariance
        self.R = np.eye(num_states) * 100
        
        # Process noise covariance
        self.Q = np.eye(num_states) * 100
        
        # Estimation error covariance
        self.S = np.eye(num_states)
        
        # Kalman gain
        # self.K = np.zeros((self.num_states, self.num_measurements))
        
        self.J_c = np.eye(num_states)
        
    def set_initial_state(self, roll, pitch):
        self.x = np.array([roll, pitch]).reshape(-1,1)
        
    def start_estimation(self):
        self.status = True
        return

    def stop_estimation(self):
        self.status = False
        return
        
    @property
    def K(self):
        # Kalman gain
        self._K = self.P @ self.J_c.T @ np.linalg.inv(self.J_c @ self.P @ self.J_c.T + self.R)
        return self._K
    
    @property
    def A(self):
        # State transition Matrix
        _A = np.zeros([self.num_states, self.num_measurements_omega])
        _A[0, 0] = 1
        _A[0, 1] = np.tan(self.x[1]) * np.sin(self.x[0])
        _A[0, 2] = np.tan(self.x[1]) * np.cos(self.x[0])
        _A[1, 1] = np.cos(self.x[0])
        _A[1, 2] = -np.sin(self.x[0])
        return _A

    def J_x(self, omega, dt=0.05):
        roll, pitch = self.x.flatten()
        J = np.zeros((self.num_states, self.num_states))
        J[0, 0] = 1 + (omega[0] + omega[1] * np.tan(pitch) * np.cos(roll) - omega[2] * np.tan(pitch) * np.sin(roll)) * dt
        J[0, 1] = (np.cos(roll) * omega[2] + np.sin(roll) * omega[1]) * dt / (np.cos(pitch) ** 2)
        J[1, 0] = (-np.sin(roll) * omega[1] - np.cos(roll) * omega[2]) * dt
        J[1, 1] = 1
        return J
            
    def state_equation(self, omega, dt=0.05):
        x_new = self.x.reshape(-1,1) + (self.A @ omega * dt).reshape(-1,1)
        return x_new
        
    def predict_step(self, omega, dt=0.05):
        J_x = self.J_x(omega, dt)
        x_hat = self.state_equation(omega, dt).reshape(-1,1)
        self.P = J_x @ self.P @ J_x.T + self.Q
        return x_hat
        
    def correct(self, acc):
        # print('z',z)
        # print('x',self.x)
        x_tilde = np.zeros(self.num_states)
        x_tilde[0] = np.arctan2(-acc[1], -acc[2])
        x_tilde[1] = np.arctan2(acc[0], np.sqrt(acc[1]**2 + acc[2]**2))
        return x_tilde.reshape(-1,1)
    
    def smooth(self, acc, omega, dt=0.05, observed=True):
        # print('before',self.x)
        x_hat = self.predict_step(omega, dt)
        # print('before2',self.x)
        if observed:
            x_tilde = self.correct(acc)
            self.x = x_hat + self.K @ (x_tilde - self.J_c @ x_hat)
            self.S = (np.eye(self.num_states) - self.K @ self.J_c) @ self.P
        # print(z-self.x[:24])
        # print('after',self.x)
        return self.x

    def loop(self, reader):
        self.status = True
        while self.status:
            sleep(self.estimation_cycle)
            # acc = reader.acc
            # omega = reader.omega
            self.smooth(reader.acc, reader.omega, dt=self.estimation_cycle_reality, observed=True)
            self._estimated_roll, self._estimated_pitch = self.state_equation(omega=reader.omega, dt=self.estimation_cycle_reality)

    @property
    def estimated_angles(self):
        return self._estimated_roll[0], self._estimated_pitch[0]
    
    @property
    def roll(self):
        return self._estimated_roll[0]
    
    @property
    def pitch(self):
        return self._estimated_pitch[0]