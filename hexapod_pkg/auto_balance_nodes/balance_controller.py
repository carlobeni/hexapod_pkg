#!/usr/bin/env python3
# balance_controller.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu 
from std_msgs.msg import String
import math
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy
)
from hexapod_pkg import config_topics_and_parameters as cfg


class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')
        cfg.check_domain_id(self.get_logger())

        # ===== QoS PARAMETERS =====
        qos2 = self._build_qos()
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.subscription = self.create_subscription(
            Imu, cfg.TOPIC_PI_PHONE_IMU_GIR_ACC, self.listener_callback, qos)
        self.publisher_ = self.create_publisher(String, cfg.TOPIC_CMD_SERIAL, qos2)

        # --- STATE VARIABLES ---
        self.angle_x = 0.0 
        self.angle_y = 0.0 
        self.angle_z = 0.0 
        
        self.last_time = self.get_clock().now()
        
        # --- SETTING 1: SOFTER FILTER ---
        # 0.98 makes it less sensitive to accelerometer vibrations
        self.alpha = 0.98  
        self.limit = 12.0 

        # --- SETTING 2: VARIABLES FOR OUTPUT SMOOTHING ---
        self.smooth_ry = 0.0
        self.smooth_rz = 0.0
        self.OUTPUT_ALPHA = 0.16  # Low value (0.1 - 0.3) = very smooth/slow movement. High value = fast reaction.
        self.DEADBAND = 0.15     # If the angle changes by less than 0.15 degrees, it does not update (avoids buzzing).
        # --- SETTING 3: CONTROL TIMER (Lower sending frequency) ---
        # Run at 20 Hz (every 0.05 seconds) instead of at the sensor speed
        self.create_timer(0.1, self.control_loop) 

        self.get_logger().info("Balance Controller: Soft Mode (20Hz Output)")

    def _build_qos(self):
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )
    
    # This callback ONLY does math (Run as fast as possible)
    def listener_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if dt <= 0 or dt > 0.1: return

        # Raw Data
        acc_x = msg.linear_acceleration.x 
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z
        
        gyro_x = msg.angular_velocity.x * 57.2958
        gyro_y = msg.angular_velocity.y * 57.2958
        gyro_z = msg.angular_velocity.z * 57.2958

        # Tilt angles (Accelerometer)
        angle_acc_y = math.atan2(acc_z, acc_x) * 57.2958
        angle_acc_z = math.atan2(acc_y, acc_x) * 57.2958

        # Complementary Filter (Fusion)
        self.angle_x += gyro_x * dt 
        self.angle_y = self.alpha * (self.angle_y + gyro_y * dt) + (1 - self.alpha) * angle_acc_y
        self.angle_z = self.alpha * (self.angle_z + gyro_z * dt) + (1 - self.alpha) * angle_acc_z

    # This callback SENDS the commands (Runs at a fixed speed, e.g., 20Hz)
    def control_loop(self):
        # 1. Obtain current target values
        target_ry = -self.angle_y  
        target_rz = self.angle_z

        # Limit target range
        target_ry = max(min(target_ry, self.limit), -self.limit)
        target_rz = max(min(target_rz, self.limit), -self.limit)

        # 2. OUTPUT SMOOTHING (Exponential Moving Average)
        # This prevents the servo from jerking. It gradually approaches the target.
        self.smooth_ry = (self.smooth_ry * (1 - self.OUTPUT_ALPHA)) + (target_ry * self.OUTPUT_ALPHA)
        self.smooth_rz = (self.smooth_rz * (1 - self.OUTPUT_ALPHA)) + (target_rz * self.OUTPUT_ALPHA)

        # 3. DEADBAND 
        # If the difference between what we want to send and what is 0 is very small, we force 0
        # Optional: This helps if the robot shakes while stationary.
        final_ry = 0.0 if abs(self.smooth_ry) < self.DEADBAND else self.smooth_ry
        final_rz = 0.0 if abs(self.smooth_rz) < self.DEADBAND else self.smooth_rz

        # 4. Send command
        comando = String()
        # We use the smoothed 'final_' values
        comando.data = f"1:ROT {final_rz:.2f} {final_ry:.2f} 0 0"
        self.publisher_.publish(comando)

def main(args=None):
    rclpy.init(args=args)
    node = BalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()