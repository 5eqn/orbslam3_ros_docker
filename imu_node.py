#!/usr/bin/env python3
import rospy
import math
import numpy as np
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
from ubt_msgs.msg import gyro_report

class ImuRepublisher:
    def __init__(self):
        rospy.init_node('imu_republisher_node', anonymous=True)

        # --- Physical Constants ---
        self.G_TO_M_PER_S2 = 9.80665
        self.DEG_TO_RAD = math.pi / 180.0
        
        # Change frame_id to camera frame, since we are aligning data to it
        self.TARGET_FRAME_ID = "cam0_optical_frame" 

        # --- Noise Parameters ---
        # Note: Rotation doesn't change noise magnitude, so these remain valid
        acc_noise_density = 0.016440119091087155
        gyro_noise_density = 0.0013994966890025684
        update_rate = 19.0

        self.acc_variance = (acc_noise_density ** 2) * update_rate
        self.gyro_variance = (gyro_noise_density ** 2) * update_rate

        # --- CALIBRATION ROTATION MATRIX (R_cam_imu) ---
        # This rotates data FROM the old IMU frame TO the Camera frame.
        # Taken from your T_cam_imu (top-left 3x3)
        self.R_cam_imu = np.array([
            [0.00301451,  0.99970157,  0.02424203],
            [0.99997196, -0.00317972,  0.00677966],
            [0.00685472,  0.02422092, -0.99968313]
        ])

        # --- Publishers & Subscribers ---
        self.pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.sub = rospy.Subscriber('/hal_gyro_report', gyro_report, self.callback)

        rospy.loginfo("IMU Aligner Started. Output data is now in Camera Frame.")

    def callback(self, msg):
        try:
            imu_msg = Imu()
            imu_msg.header.stamp = rospy.Time.now()
            imu_msg.header.frame_id = self.TARGET_FRAME_ID

            # --- STEP 1: Extract "Old" Vectors (Previous Logic) ---
            # We preserve your original mapping logic first to reconstruct 
            # exactly what Kalibr saw as "imu0".
            
            # Acceleration (Your original RFD -> FLU mapping)
            raw_acc = np.array([
                msg.accel_data[1] * self.G_TO_M_PER_S2,  # x_ros
                -msg.accel_data[0] * self.G_TO_M_PER_S2, # y_ros
                -msg.accel_data[2] * self.G_TO_M_PER_S2  # z_ros
            ])

            # Angular Velocity (Your original LBH -> FLU mapping)
            raw_gyro = np.array([
                -msg.gyro_data[1] * self.DEG_TO_RAD, # x_ros
                msg.gyro_data[0] * self.DEG_TO_RAD,  # y_ros
                msg.gyro_data[2] * self.DEG_TO_RAD   # z_ros
            ])

            # --- STEP 2: Apply Calibration Rotation ---
            # Rotates the vector so it aligns with the Camera Axes
            # v_cam = R_cam_imu * v_imu
            aligned_acc = self.R_cam_imu.dot(raw_acc)
            aligned_gyro = self.R_cam_imu.dot(raw_gyro)

            # --- STEP 3: Populate Message ---
            
            imu_msg.linear_acceleration = Vector3(*aligned_acc)
            imu_msg.angular_velocity = Vector3(*aligned_gyro)

            # --- STEP 4: Handle Orientation (Quaternion) ---
            if hasattr(msg, 'euler_data') and len(msg.euler_data) >= 3:
                # 1. Get original Quaternion (in Old IMU Frame)
                ros_roll  = -msg.euler_data[1] * self.DEG_TO_RAD 
                ros_pitch =  msg.euler_data[0] * self.DEG_TO_RAD 
                ros_yaw   =  msg.euler_data[2] * self.DEG_TO_RAD 
                q_original = tf.transformations.quaternion_from_euler(ros_roll, ros_pitch, ros_yaw)

                # 2. Convert Calibration Matrix to Quaternion
                # This represents the rotation from Old IMU -> Camera
                T_calib = np.identity(4)
                T_calib[0:3, 0:3] = self.R_cam_imu
                q_calib = tf.transformations.quaternion_from_matrix(T_calib)

                # 3. Rotate the measurement: q_new = q_calib * q_original
                q_aligned = tf.transformations.quaternion_multiply(q_calib, q_original)

                imu_msg.orientation = Quaternion(*q_aligned)
                
                # Orientation covariance
                imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01]
            else:
                imu_msg.orientation = Quaternion(0, 0, 0, 1)
                imu_msg.orientation_covariance[0] = -1.0

            # --- STEP 5: Covariances ---
            # Technically, rotating the frame rotates the covariance matrix too: 
            # Cov_new = R * Cov_old * R.T
            # Since your covariance is diagonal and isotropic (same for x,y,z), 
            # rotation doesn't change the values. We can keep the diagonal logic.
            
            imu_msg.linear_acceleration_covariance = [
                self.acc_variance, 0.0, 0.0,
                0.0, self.acc_variance, 0.0,
                0.0, 0.0, self.acc_variance
            ]

            imu_msg.angular_velocity_covariance = [
                self.gyro_variance, 0.0, 0.0,
                0.0, self.gyro_variance, 0.0,
                0.0, 0.0, self.gyro_variance
            ]

            self.pub.publish(imu_msg)

        except Exception as e:
            rospy.logerr_throttle(1, f"Conversion error: {e}")

if __name__ == '__main__':
    try:
        node = ImuRepublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
