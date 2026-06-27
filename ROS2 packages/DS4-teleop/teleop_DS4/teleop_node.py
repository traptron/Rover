#!/usr/bin/env python3
"""
Teleoperation node for 6-wheel rover using DualShock 4 controller.

This node subscribes to joy_node output and publishes velocity commands
to the rover's motor controller via cmd_vel topic.

Control mapping:
- R2 trigger: forward speed (normalized [0, 1])
- L2 trigger: reverse/braking speed (normalized [0, 1])
- Left stick X axis: turning (angular velocity [-1, 1])

Safety features:
- Watchdog timer: stops rover if no joy message for 200ms
- Deadzone: ignores inputs below threshold (~0.05)
- Smoothing: rate limiter prevents sudden acceleration
- Clamping: ensures output stays within [-1.0, 1.0]
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import time

# ============================================================================
# CONSTANTS - Control Mapping
# ============================================================================
# DualShock 4 axis indices (via joy_node)
AXIS_LEFT_STICK_X = 0
AXIS_LEFT_STICK_Y = 1
AXIS_RIGHT_STICK_X = 2
AXIS_RIGHT_STICK_Y = 3
AXIS_L2_TRIGGER = 4      # Range: [1.0, -1.0]
AXIS_R2_TRIGGER = 5      # Range: [1.0, -1.0]


class TeleopDS4Node(Node):
    """Teleoperation node for DualShock 4 controller."""

    def __init__(self):
        super().__init__('teleop_ds4_node')

        # ====================================================================
        # PARAMETERS
        # ====================================================================
        self.declare_parameter('max_linear_speed', 0.6)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('deadzone', 0.05)
        self.declare_parameter('accel_limit', 0.5)  # units/second
        self.declare_parameter('watchdog_timeout', 0.2)  # seconds
        self.declare_parameter('debug', False)

        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.deadzone = self.get_parameter('deadzone').value
        self.accel_limit = self.get_parameter('accel_limit').value
        self.watchdog_timeout = self.get_parameter('watchdog_timeout').value
        self.debug = self.get_parameter('debug').value

        # ====================================================================
        # STATE VARIABLES
        # ====================================================================
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0
        self.last_joy_time = time.time()
        self.watchdog_active = False

        # ====================================================================
        # PUBLISHERS / SUBSCRIBERS
        # ====================================================================
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.joy_sub = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

        # ====================================================================
        # TIMERS
        # ====================================================================
        # Main control loop at 50 Hz
        self.create_timer(0.02, self.control_loop)

        # Watchdog timer at 100 Hz (frequent checks)
        self.create_timer(0.01, self.watchdog_check)

        # Debug logger at 5 Hz (throttled)
        self.create_timer(0.2, self.debug_log)

        self.get_logger().info('Teleoperation node initialized')
        self.get_logger().info(
            f'Params: max_linear={self.max_linear_speed}, '
            f'max_angular={self.max_angular_speed}, '
            f'deadzone={self.deadzone}, '
            f'accel_limit={self.accel_limit}'
        )

    # ========================================================================
    # JOY MESSAGE CALLBACK
    # ========================================================================
    def joy_callback(self, msg: Joy) -> None:
        """
        Process incoming joy message and extract command.

        Args:
            msg: sensor_msgs/Joy message from joy_node
        """
        self.last_joy_time = time.time()

        # Validate message has required axes
        if len(msg.axes) < 6:
            self.get_logger().warn(
                f'Invalid joy message: expected >= 6 axes, got {len(msg.axes)}'
            )
            return

        # Extract and normalize triggers
        try:
            l2_raw = msg.axes[AXIS_L2_TRIGGER]
            r2_raw = msg.axes[AXIS_R2_TRIGGER]
            left_stick_x = msg.axes[AXIS_LEFT_STICK_X]

            # Normalize triggers from [1, -1] to [0, 1]
            l2_normalized = self._normalize_trigger(l2_raw)
            r2_normalized = self._normalize_trigger(r2_raw)

            # Compute desired velocities
            desired_linear = (r2_normalized - l2_normalized) * self.max_linear_speed
            desired_angular = left_stick_x * self.max_angular_speed

            # Apply deadzone
            desired_linear = self._apply_deadzone(desired_linear)
            desired_angular = self._apply_deadzone(desired_angular)

            # Clamp to valid range
            desired_linear = self._clamp(desired_linear, -self.max_linear_speed, self.max_linear_speed)
            desired_angular = self._clamp(desired_angular, -self.max_angular_speed, self.max_angular_speed)

            # Apply smoothing (rate limiter)
            self.current_linear_x = self._smooth_value(
                self.current_linear_x,
                desired_linear,
                self.accel_limit * 0.02  # 0.02 is control loop period
            )
            self.current_angular_z = self._smooth_value(
                self.current_angular_z,
                desired_angular,
                self.accel_limit * 0.02
            )

            self.watchdog_active = False

            if self.debug:
                self.get_logger().debug(
                    f'Joy: L2={l2_normalized:.2f} R2={r2_normalized:.2f} '
                    f'LX={left_stick_x:.2f} => '
                    f'linear={self.current_linear_x:.2f} angular={self.current_angular_z:.2f}'
                )

        except Exception as e:
            self.get_logger().error(f'Error processing joy message: {e}')

    # ========================================================================
    # MAIN CONTROL LOOP
    # ========================================================================
    def control_loop(self) -> None:
        """Main control loop - publishes cmd_vel at 50 Hz."""
        twist = Twist()
        twist.linear.x = self.current_linear_x
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.current_angular_z

        self.cmd_vel_pub.publish(twist)

    # ========================================================================
    # WATCHDOG
    # ========================================================================
    def watchdog_check(self) -> None:
        """
        Check if joy messages are being received.
        If no message for > watchdog_timeout, send zero velocity.
        """
        elapsed = time.time() - self.last_joy_time

        if elapsed > self.watchdog_timeout and not self.watchdog_active:
            self.get_logger().warn(
                f'Watchdog timeout: no joy message for {elapsed:.3f}s'
            )
            self.current_linear_x = 0.0
            self.current_angular_z = 0.0
            self.watchdog_active = True

    # ========================================================================
    # DEBUG LOGGING (THROTTLED)
    # ========================================================================
    def debug_log(self) -> None:
        """Log current state at 5 Hz."""
        if not self.debug:
            return

        elapsed = time.time() - self.last_joy_time
        self.get_logger().debug(
            f'State: linear={self.current_linear_x:.3f} '
            f'angular={self.current_angular_z:.3f} '
            f'watchdog_age={elapsed:.3f}s'
        )

    # ========================================================================
    # UTILITY FUNCTIONS
    # ========================================================================
    @staticmethod
    def _normalize_trigger(value: float) -> float:
        """
        Convert trigger value from [-1, 1] to [0, 1].

        Trigger ranges from 1.0 (released) to -1.0 (fully pressed).
        Formula: normalized = (1 - value) / 2

        Args:
            value: Raw trigger value in [-1, 1]

        Returns:
            Normalized value in [0, 1]
        """
        return (1.0 - value) / 2.0

    def _apply_deadzone(self, value: float) -> float:
        """
        Apply deadzone threshold to ignore small inputs.

        Args:
            value: Input value

        Returns:
            Value with deadzone applied (0 if below threshold)
        """
        if abs(value) < self.deadzone:
            return 0.0
        return value

    @staticmethod
    def _clamp(value: float, min_val: float, max_val: float) -> float:
        """
        Clamp value to range [min_val, max_val].

        Args:
            value: Input value
            min_val: Minimum allowed value
            max_val: Maximum allowed value

        Returns:
            Clamped value
        """
        return max(min_val, min(max_val, value))

    @staticmethod
    def _smooth_value(current: float, desired: float, max_delta: float) -> float:
        """
        Apply rate limiting (smoothing) to prevent sudden jumps.

        This implements a simple acceleration limit.

        Args:
            current: Current velocity
            desired: Desired velocity
            max_delta: Maximum change per timestep

        Returns:
            New velocity (limited change from current)
        """
        delta = desired - current
        if abs(delta) <= max_delta:
            return desired
        return current + (max_delta if delta > 0 else -max_delta)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = TeleopDS4Node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down teleoperation node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
