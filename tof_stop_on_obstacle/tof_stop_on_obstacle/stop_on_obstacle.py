#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern
from std_msgs.msg import ColorRGBA


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def make_led_pattern(r: float, g: float, b: float, count: int) -> LEDPattern:
    msg = LEDPattern()
    msg.rgb_vals = []
    for _ in range(count):
        c = ColorRGBA()
        c.r, c.g, c.b, c.a = float(r), float(g), float(b), 1.0
        msg.rgb_vals.append(c)
    return msg


class StopOnObstacle(Node):
    """
    Initial program:
      - if ToF range < stop_distance: STOP + RED LEDs
      - else: move forward + GREEN LEDs
      - if no range received yet: STOP (safety)
    """

    def __init__(self):
        super().__init__("stop_on_obstacle")

        # ---- parameters (change via --ros-args -p ...) ----
        self.declare_parameter("robot_name", "duckie02")
        self.declare_parameter("stop_distance", 0.25)   # meters
        self.declare_parameter("forward_speed", 0.25)   # [-1..1]
        self.declare_parameter("led_count", 5)          # from your manual

        robot = self.get_parameter("robot_name").value
        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.led_count = int(self.get_parameter("led_count").value)

        self.forward_speed = clamp(self.forward_speed, -1.0, 1.0)

        # ---- topics ----
        self.range_topic = f"/{robot}/range"
        self.wheels_topic = f"/{robot}/wheels_cmd"
        self.led_topic = f"/{robot}/led_pattern"

        # ---- ROS2 pub/sub ----
        self.range_sub = self.create_subscription(Range, self.range_topic, self.on_range, 10)
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, self.wheels_topic, 10)
        self.led_pub = self.create_publisher(LEDPattern, self.led_topic, 10)

        self.last_range = None
        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

        self.get_logger().info("✅ StopOnObstacle node started")
        self.get_logger().info(f"  range:  {self.range_topic}")
        self.get_logger().info(f"  wheels: {self.wheels_topic}")
        self.get_logger().info(f"  leds:   {self.led_topic}")
        self.get_logger().info(f"  stop_distance={self.stop_distance}m forward_speed={self.forward_speed} led_count={self.led_count}")

    def on_range(self, msg: Range):
        # ignore invalid readings
        if msg.range <= 0.0:
            return
        self.last_range = msg.range

    def publish_wheels(self, left: float, right: float):
        cmd = WheelsCmdStamped()
        cmd.vel_left = float(clamp(left, -1.0, 1.0))
        cmd.vel_right = float(clamp(right, -1.0, 1.0))
        self.wheels_pub.publish(cmd)

    def publish_leds(self, r: float, g: float, b: float):
        # LEDs are "nice to have"; if LED msg type differs, don't crash movement
        try:
            self.led_pub.publish(make_led_pattern(r, g, b, self.led_count))
        except Exception as e:
            self.get_logger().warn(f"LED publish failed (still driving OK): {e}")

    def loop(self):
        # Safety: no sensor data yet => stop
        if self.last_range is None:
            self.publish_wheels(0.0, 0.0)
            return

        if self.last_range < self.stop_distance:
            self.publish_wheels(0.0, 0.0)
            self.publish_leds(1.0, 0.0, 0.0)   # red
        else:
            self.publish_wheels(self.forward_speed, self.forward_speed)
            self.publish_leds(0.0, 1.0, 0.0)   # green


def main():
    rclpy.init()
    node = StopOnObstacle()
    try:
        rclpy.spin(node)
    finally:
        node.publish_wheels(0.0, 0.0)  # stop on exit
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
