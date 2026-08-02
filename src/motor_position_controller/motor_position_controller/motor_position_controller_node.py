#!/usr/bin/env python3
"""Interactive bounded position-reference publisher for two mechanism axes."""

from __future__ import annotations

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class MotorPositionControllerNode(Node):
    """Publish operator-defined position references at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("motor_position_controller")
        self.motor1_pub = self.create_publisher(Float32, "/motor1_position", 10)
        self.motor2_pub = self.create_publisher(Float32, "/motor2_position", 10)
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.motor1_position = 0.0
        self.motor2_position = 0.0
        self.max_position = 12.56
        self.min_position = -12.56

        self.get_logger().info("Motor position command publisher started.")
        thread = threading.Thread(target=self.keyboard_input_thread, daemon=True)
        thread.start()

    def clamp(self, value: float) -> float:
        return max(min(value, self.max_position), self.min_position)

    def timer_callback(self) -> None:
        motor1 = Float32()
        motor1.data = self.motor1_position
        self.motor1_pub.publish(motor1)

        motor2 = Float32()
        motor2.data = self.motor2_position
        self.motor2_pub.publish(motor2)

    def set_position(self, target: str, value: float) -> None:
        value = self.clamp(value)
        if target in {"both", "motor1"}:
            self.motor1_position = value
        if target in {"both", "motor2"}:
            self.motor2_position = value
        self.get_logger().info(f"Updated {target} target to {value:.2f} rad")

    def keyboard_input_thread(self) -> None:
        self.get_logger().info(
            "Commands: T <value>, T1 <value>, T2 <value>, or q to quit."
        )
        while rclpy.ok():
            try:
                command = input().strip()
                if command == "q":
                    rclpy.shutdown()
                    return

                prefix, raw_value = command.split(maxsplit=1)
                target = {"T": "both", "T1": "motor1", "T2": "motor2"}.get(prefix)
                if target is None:
                    raise ValueError("unknown command")
                self.set_position(target, float(raw_value))
            except (EOFError, KeyboardInterrupt):
                rclpy.shutdown()
                return
            except (ValueError, TypeError):
                self.get_logger().warning(
                    "Invalid command. Use T, T1 or T2 followed by a numeric value."
                )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MotorPositionControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
