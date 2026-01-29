#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Range
from std_msgs.msg import Header, ColorRGBA
from duckietown_msgs.msg import WheelsCmdStamped, LEDPattern


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


class AvoidObstaclesBlink(Node):
    """
    Combined behavior:
      - Drive forward with SOLID GREEN LEDs
      - If obstacle closer than stop_distance:
          * stop briefly
          * turn in place for turn_time seconds
          * while turning: BLINK RED LEDs
      - Then continue forward
    """

    def __init__(self):
        super().__init__("avoid_obstacles_blink")

        # -------- Parameters --------
        self.declare_parameter("robot_name", "duckie03")

        self.declare_parameter("stop_distance", 0.25)     # meters
        self.declare_parameter("forward_speed", 0.25)     # [-1..1]
        self.declare_parameter("turn_speed", 0.25)        # [-1..1]
        self.declare_parameter("turn_time", 0.8)          # seconds
        self.declare_parameter("turn_direction", "left")  # "left" or "right"

        # LED params
        self.declare_parameter("led_count", 5)            # from your manual
        self.declare_parameter("blink_hz", 2.0)           # red blink frequency while turning

        robot = self.get_parameter("robot_name").value

        self.stop_distance = float(self.get_parameter("stop_distance").value)
        self.forward_speed = clamp(float(self.get_parameter("forward_speed").value), -1.0, 1.0)
        self.turn_speed = clamp(float(self.get_parameter("turn_speed").value), -1.0, 1.0)
        self.turn_time = float(self.get_parameter("turn_time").value)
        self.turn_direction = str(self.get_parameter("turn_direction").value).lower()

        self.led_count = int(self.get_parameter("led_count").value)
        self.blink_hz = float(self.get_parameter("blink_hz").value)
        if self.blink_hz <= 0:
            self.blink_hz = 2.0

        # -------- Topics --------
        self.range_topic = f"/{robot}/range"
        self.wheels_topic = f"/{robot}/wheels_cmd"
        self.led_topic = f"/{robot}/led_pattern"

        # -------- Pub/Sub --------
        self.tof_sub = self.create_subscription(Range, self.range_topic, self.on_range, 10)
        self.wheels_pub = self.create_publisher(WheelsCmdStamped, self.wheels_topic, 10)
        self.led_pub = self.create_publisher(LEDPattern, self.led_topic, 10)

        # -------- State --------
        self.last_range = None

        self.state = "FORWARD"      # "FORWARD" or "TURN"
        self.turn_until = 0.0

        # blink control (used in TURN)
        self.blink_on = True
        self.next_blink_toggle = time.monotonic()
        self.blink_half_period = 0.5 / self.blink_hz  # seconds (on/off toggle interval)

        # Control loop
        self.timer = self.create_timer(0.1, self.loop)  # 10 Hz

        self.get_logger().info("✅ AvoidObstaclesBlink started")
        self.get_logger().info(f"  range:  {self.range_topic}")
        self.get_logger().info(f"  wheels: {self.wheels_topic}")
        self.get_logger().info(f"  leds:   {self.led_topic}")
        self.get_logger().info(
            f"  stop_distance={self.stop_distance} forward_speed={self.forward_speed} "
            f"turn_speed={self.turn_speed} turn_time={self.turn_time}s dir={self.turn_direction} "
            f"blink_hz={self.blink_hz}"
        )

    def on_range(self, msg: Range):
        # ignore invalid
        if msg.range <= 0.0:
            return
        self.last_range = msg.range

    def publish_wheels(self, frame_id: str, left: float, right: float):
        msg = WheelsCmdStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.vel_left = float(clamp(left, -1.0, 1.0))
        msg.vel_right = float(clamp(right, -1.0, 1.0))
        self.wheels_pub.publish(msg)

    def publish_leds_safe(self, r: float, g: float, b: float):
        # LEDs should never crash driving
        try:
            self.led_pub.publish(make_led_pattern(r, g, b, self.led_count))
        except Exception as e:
            self.get_logger().warn(f"LED publish failed (movement continues): {e}")

    def set_state(self, new_state: str):
        if new_state == self.state:
            return
        self.state = new_state

        # When entering TURN, reset blink so it starts red immediately
        if self.state == "TURN":
            now = time.monotonic()
            self.blink_on = True
            self.next_blink_toggle = now + self.blink_half_period

    def loop(self):
        now = time.monotonic()

        # Safety: no ToF yet -> stop + (optional) keep green off
        if self.last_range is None:
            self.publish_wheels("no_range", 0.0, 0.0)
            return

        # -------- TURN state: blink red + turn --------
        if self.state == "TURN":
            # stop turning after time is up
            if now >= self.turn_until:
                self.set_state("FORWARD")
                return

            # Blink logic: toggle red <-> off
            if now >= self.next_blink_toggle:
                self.blink_on = not self.blink_on
                self.next_blink_toggle = now + self.blink_half_period

            if self.blink_on:
                self.publish_leds_safe(1.0, 0.0, 0.0)  # red
            else:
                self.publish_leds_safe(0.0, 0.0, 0.0)  # off

            # turn in place
            if self.turn_direction == "right":
                self.publish_wheels("turn_right", self.turn_speed, -self.turn_speed)
            else:
                self.publish_wheels("turn_left", -self.turn_speed, self.turn_speed)
            return

        # -------- FORWARD state: solid green + drive unless obstacle --------
        if self.last_range < self.stop_distance:
            # obstacle detected -> stop then turn
            self.publish_wheels("stop_before_turn", 0.0, 0.0)

            # enter TURN
            self.turn_until = now + self.turn_time
            self.set_state("TURN")
            return

        # clear -> drive forward + green
        self.publish_leds_safe(0.0, 1.0, 0.0)  # green
        self.publish_wheels("forward", self.forward_speed, self.forward_speed)


def main():
    rclpy.init()
    node = AvoidObstaclesBlink()
    try:
        rclpy.spin(node)
    finally:
        node.publish_wheels("shutdown", 0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
