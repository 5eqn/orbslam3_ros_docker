#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import Imu, Image

class SyncMonitor:
    def __init__(self):
        rospy.init_node('sensor_sync_monitor', anonymous=True)
        
        # Variables to store the latest timestamps
        self.last_imu_ts = None
        self.last_cam_ts = None
        
        # Counters to check if data is actually flowing
        self.imu_count = 0
        self.cam_count = 0

        # Subscribers
        rospy.Subscriber('/imu/data', Imu, self.imu_cb)
        rospy.Subscriber('/camera/image_raw', Image, self.cam_cb)

        rospy.loginfo("Waiting for messages on /imu/data and /camera/image_raw...")

    def imu_cb(self, msg):
        self.last_imu_ts = msg.header.stamp
        self.imu_count += 1

    def cam_cb(self, msg):
        self.last_cam_ts = msg.header.stamp
        self.cam_count += 1

    def run(self):
        rate = rospy.Rate(10) # Update the display at 10Hz (data might come faster)
        
        print(f"{'WALL CLOCK':<15} | {'IMU TIMESTAMP':<18} | {'CAM TIMESTAMP':<18} | {'DELTA (Cam-Imu)':<18}")
        print("-" * 75)

        while not rospy.is_shutdown():
            if self.last_imu_ts is not None and self.last_cam_ts is not None:
                # Convert to seconds for calculation
                t_imu = self.last_imu_ts.to_sec()
                t_cam = self.last_cam_ts.to_sec()
                
                # Calculate difference
                delta = t_cam - t_imu
                
                # Get current wall time for reference
                wall_time = rospy.Time.now().to_sec()

                # Format output strings
                # Showing last 4 digits of timestamp for cleaner reading
                imu_str = f"{t_imu:.6f}"
                cam_str = f"{t_cam:.6f}"
                
                # visual indicator for delta
                delta_str = f"{delta:.6f}s"
                
                # Print using Carriage Return (\r) to stay on the same line (optional)
                # or print new lines to see history. Here we print new lines to track drift.
                print(f"{wall_time:.4f}      | {imu_str}       | {cam_str}       | {delta_str}")
                
            else:
                # Waiting for data
                status = []
                if self.last_imu_ts is None: status.append("No IMU")
                if self.last_cam_ts is None: status.append("No CAM")
                print(f"Waiting for data... ({', '.join(status)})", end='\r')

            rate.sleep()

if __name__ == '__main__':
    try:
        monitor = SyncMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
