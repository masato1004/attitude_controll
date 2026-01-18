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
    roll, pitch = reader.acc2euler(acc[0], acc[1], acc[2])
    ekf.set_initial_state(roll, pitch)
    ekf.start_estimation()
    
    # kalman filter loop threading
    thread_ekf = threading.Thread(target=ekf.loop, args=(reader,), daemon=True)
    thread_ekf.start()
    
    # controller loop threading
    thread_controller = threading.Thread(target=att_ctrl.loop, args=(ekf,))
    thread_controller.start()
    
    # communicator instance
    com = TcpCommunicator(ip="192.168.1.228", port=50001, data_num=2, seperator="_")
    com.connect()
    thread_communicator = threading.Thread(target=com.loop_send, args=(ekf,))
    thread_communicator.start()
    
    # print("Starting main loop...")
    
    sleep(1)
    
    for i in range(500):
        sleep(0.02)
        now = time()
        
        # estimated_roll, estimated_pitch = ekf.estimated_angles
        # controll_points = att_ctrl.calculate_pillars_point(estimated_roll, estimated_pitch)
        # att_ctrl.calculate_arm_angle(ekf)
        
        # com.send(data=[estimated_roll, estimated_pitch], starts_with='s')
        
        if i % 2 == 0:
            print(att_ctrl.target_angle_differential)
        
        # if i % 2 == 0:
        #     update = True
        #     realtime_viewer.plot_point(controll_points)
        # else:
        #     update = False
        
        # realtime_viewer.add_data(estimated_roll, estimated_pitch, now, update=update)
    
    reader.stop()
    ekf.stop_estimation()
    att_ctrl.stop_controller()
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
