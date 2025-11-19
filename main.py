from motion_reader import MotionReader
from kalman_filter import ExtendedKalmanFilter as EKF
from viewer import AngleViewer
from ios_viewer import RealTimeViewerIOS, RealTime3DViewerIOS
from attitude_controller import AttitudeController as AttCtrl
from time import time, sleep
import threading
import numpy as np
import matplotlib.pyplot as plt
import ui

def main():
    reader = MotionReader(sleep_time=0.01)
    reader.start()
    
    att_ctrl = AttCtrl()
    
    # viewer_acc = AngleViewer(name='ACC')
    # viewer_gyro = AngleViewer(name='GYRO')
    # # viewer_omega = AngleViewer(name='OMEGA')
    # viewer_ekf = AngleViewer(name='EKF')
    # viewer_error = AngleViewer(name='ERROR')
    
    # realtime_viewer = RealTimeViewerIOS(name='Real-Time EKF')
    realtime_viewer = RealTime3DViewerIOS(name='Real-Time EKF')
    realtime_viewer.present('sheet')
    
    ekf = EKF()
    
    thread_motion_reader = threading.Thread(target=reader.loop, daemon=True)  # Explicitly pass loop duration
    thread_motion_reader.start()
    
    acc = reader.acc
    roll, pitch = reader.acc2euler(-acc[1], -acc[0], acc[2])
    ekf.set_initial_state(roll, pitch)
    ekf.start_estimation()
    
    thread_ekf = threading.Thread(target=ekf.loop, args=(reader,), daemon=True)
    thread_ekf.start()
    
    # print("Starting main loop...")
    
    for i in range(200):
        sleep(0.05)
        now = time()
        # acc = reader.acc
        # gyro = reader.gyro
        # mag = reader.mag
        # omega = reader.omega
        # estimated_roll, estimated_pitch = ekf.smooth([-acc[1], -acc[0], acc[2]], omega, dt=0.05, observed=True)
        estimated_roll, estimated_pitch = ekf.get_estimated_angles()
        controll_points = att_ctrl.calculate_pillars_point(estimated_roll, estimated_pitch)
        # roll, pitch = reader.acc2euler(-acc[1], -acc[0], acc[2])
        # viewer_acc.add_data(roll, pitch, now)
        # viewer_gyro.add_data(gyro[0], gyro[1], now)
        # viewer_omega.add_data(omega[0], omega[1], now)
        # viewer_ekf.add_data(estimated_roll, estimated_pitch, now)
        # viewer_error.add_data(roll - estimated_roll, pitch - estimated_pitch, now)
        if i % 2 == 0:
            update = True
            realtime_viewer.plot_point(controll_points)
        else:
            update = False
        # realtime_viewer.add_data(roll, pitch, now, update=update)
        realtime_viewer.add_data(estimated_roll, estimated_pitch, now, update=update)
        #     print('---')
        #     print(f"Acceleration: X={acc[0]:.3f}, Y={acc[1]:.3f}, Z={acc[2]:.3f}")
        #     print(f"Gyro: Roll={gyro[0]:.3f}, Pitch={gyro[1]:.3f}, Yaw={gyro[2]:.3f}")
        #     print(f"Magnetic Field: X={mag[0]:.3f}, Y={mag[1]:.3f}, Z={mag[2]:.3f}")
    
    reader.stop()
    ekf.stop_estimation()
    
    # plt.figure(figsize=(10, 5))
    # viewer_acc.plot()
    # viewer_gyro.plot()
    # viewer_omega.plot()
    # viewer_ekf.plot()
    # viewer_error.plot()
    # plt.show()

if __name__ == '__main__':
    main()
