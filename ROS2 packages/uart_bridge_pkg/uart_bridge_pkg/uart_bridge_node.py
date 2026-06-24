#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial
import threading
import struct
import math
import time

def calculate_crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def yaw_to_quaternion(yaw):
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]

class UartBridgeNode(Node):
    def __init__(self):
        super().__init__('uart_bridge_node')
        
        # Declare parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.0)
        self.declare_parameter('motor_id', 0)
        self.declare_parameter('serial_port', '/dev/ttyS7')
        self.declare_parameter('baudrate', 115200)

        # Odometry variables
        self.prev_encoders = None
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Constants
        self.TRACK_WIDTH = 0.29
        self.METERS_PER_TICK = (math.pi * 0.10) / (46.0 * 44.0)

        # Connect to serial port
        port = self.get_parameter('serial_port').value
        baudrate = self.get_parameter('baudrate').value
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1.0)
            self.get_logger().info(f"Connected to {port} at {baudrate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to serial port: {e}")
            raise e

        # Parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)

        # Publishers and Subscribers
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_cb, 10)

        # Start RX thread
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

    def parameters_callback(self, params):
        updated = False
        kp = self.get_parameter('kp').value
        ki = self.get_parameter('ki').value
        kd = self.get_parameter('kd').value
        motor_id = self.get_parameter('motor_id').value
        
        for param in params:
            if param.name == 'kp':
                kp = float(param.value)
                updated = True
            elif param.name == 'ki':
                ki = float(param.value)
                updated = True
            elif param.name == 'kd':
                kd = float(param.value)
                updated = True
            elif param.name == 'motor_id':
                motor_id = int(param.value)
                updated = True
        
        if updated:
            self.send_pid_tune(motor_id, kp, ki, kd)
            self.get_logger().info(f"PID Tune Sent - Motor {motor_id}: Kp={kp}, Ki={ki}, Kd={kd}")
        
        return SetParametersResult(successful=True)

    def send_pid_tune(self, motor_id, kp, ki, kd):
        # Pack PID Tune message (msg_id=2)
        # Header (2), msg_id (1), motor_id (1), kp (4), ki (4), kd (4), reserved (7)
        payload = struct.pack('<H B B f f f 7s', 0xAA55, 2, motor_id, kp, ki, kd, b'\x00'*7)
        crc = calculate_crc8(payload)
        packet = payload + struct.pack('<B', crc)
        
        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write to serial: {e}")

    def cmd_vel_cb(self, msg):
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        led_mask = 1 # Optional: set LED mask behavior based on cmd_vel or other state
        
        # Pack Movement message (msg_id=1)
        # Header (2), msg_id (1), linear_x (4), angular_z (4), led_mask (1), reserved (11)
        payload = struct.pack('<H B f f B 11s', 0xAA55, 1, linear_x, angular_z, led_mask, b'\x00'*11)
        crc = calculate_crc8(payload)
        packet = payload + struct.pack('<B', crc)
        
        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to write cmd_vel to serial: {e}")

    def rx_loop(self):
        while rclpy.ok():
            try:
                sync = self.ser.read(1)
                if sync == b'\xBB':
                    sync2 = self.ser.read(1)
                    if sync2 == b'\xBB':
                        # Read 24 bytes of encoders + 1 byte CRC
                        data = self.ser.read(25)
                        if len(data) == 25:
                            packet_except_crc = b'\xBB\xBB' + data[:-1]
                            crc_recv = data[-1]
                            
                            if calculate_crc8(packet_except_crc) == crc_recv:
                                encoders = struct.unpack('<6i', data[:-1])
                                self.update_odometry(encoders)
                            else:
                                self.get_logger().warn("UART RX CRC Error")
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f"Unexpected error in rx_loop: {e}")
                time.sleep(1.0)

    def update_odometry(self, encoders):
        if self.prev_encoders is None:
            self.prev_encoders = encoders
            return

        delta_encoders = [encoders[i] - self.prev_encoders[i] for i in range(6)]
        self.prev_encoders = encoders

        # Left encoders: 0, 1, 2. Right encoders: 3, 4, 5.
        left_ticks = (delta_encoders[0] + delta_encoders[1] + delta_encoders[2]) / 3.0
        right_ticks = (delta_encoders[3] + delta_encoders[4] + delta_encoders[5]) / 3.0

        dist_left = left_ticks * self.METERS_PER_TICK
        dist_right = right_ticks * self.METERS_PER_TICK

        dist = (dist_right + dist_left) / 2.0
        d_theta = (dist_right - dist_left) / self.TRACK_WIDTH

        self.x += dist * math.cos(self.theta + d_theta / 2.0)
        self.y += dist * math.sin(self.theta + d_theta / 2.0)
        self.theta += d_theta

        # Publish Odometry
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        q = yaw_to_quaternion(self.theta)
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]

        self.odom_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UartBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
