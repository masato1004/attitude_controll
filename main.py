from motion_reader import MotionReader
from kalman_filter import ExtendedKalmanFilter as EKF
# from viewer import AngleViewer
from ios_viewer import RealTimeViewerIOS, RealTime3DViewerIOS
from attitude_controller import AttitudeController as AttCtrl
from time import time, sleep
import threading
import numpy as np
import matplotlib.pyplot as plt
import ui

from communicator import TcpCommunicator

def main():
    # realtime_viewer = RealTime3DViewerIOS(name='Real-Time EKF')
    # realtime_viewer.present('sheet')
    
    
    # read motion
    reader = MotionReader(sleep_time=0.01)
    reader.start()
    
    # attitude control instance
    att_ctrl = AttCtrl()
    
    # kalman filter instance
    ekf = EKF()
    
    # motion read loop threading
    thread_motion_reader = threading.Thread(target=reader.loop, daemon=True)  # Explicitly pass loop duration
    thread_motion_reader.start()
    
    # initial estimation
    acc = reader.acc
    roll, pitch = reader.acc2euler(acc[1], -acc[0], acc[2])
    ekf.set_initial_state(roll, pitch)
    ekf.start_estimation()
    
    # kalman filter loop threading
    thread_ekf = threading.Thread(target=ekf.loop, args=(reader,), daemon=True)
    thread_ekf.start()
    
    # communicator instance
    com = TcpCommunicator(ip="192.168.1.228", port=50001)
    com.connect()
    
    # print("Starting main loop...")
    
    sleep(1)
    
    for i in range(200):
        sleep(0.03)
        now = time()
        
        estimated_roll, estimated_pitch = ekf.get_estimated_angles()
        controll_points = att_ctrl.calculate_pillars_point(estimated_roll, estimated_pitch)
        
        com.send(f"s{estimated_roll[0]}_{estimated_pitch[0]}")
        
        # if i % 2 == 0:
        #     update = True
        #     realtime_viewer.plot_point(controll_points)
        # else:
        #     update = False
        
        # realtime_viewer.add_data(estimated_roll, estimated_pitch, now, update=update)
    
    reader.stop()
    ekf.stop_estimation()
    com.close()
    
    # plt.figure(figsize=(10, 5))
    # viewer_acc.plot()
    # viewer_gyro.plot()
    # viewer_omega.plot()
    # viewer_ekf.plot()
    # viewer_error.plot()
    # plt.show()

if __name__ == '__main__':
    main()
