import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelWatchdog(Node):
    def __init__(self):
        super().__init__('cmd_vel_watchdog')
        self.declare_parameter('max_linear_vel', 0.20)
        self.declare_parameter('max_angular_vel', 1.00)
        self.declare_parameter('watchdog_timeout_sec', 0.25)
        self.declare_parameter('publish_rate_hz', 20.0)

        self.max_lin = float(self.get_parameter('max_linear_vel').value)
        self.max_ang = float(self.get_parameter('max_angular_vel').value)
        self.timeout_sec = float(self.get_parameter('watchdog_timeout_sec').value)
        rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.last_cmd = Twist()
        self.last_cmd_time = None
        self.timed_out = True

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.timer_callback)

    def cmd_callback(self, msg: Twist):
        vals = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        if not all(math.isfinite(v) for v in vals):
            self.get_logger().warn('Rejected non-finite Twist command; zeroing output.')
            self.last_cmd = Twist()
            self.timed_out = True
            self.pub.publish(Twist())
            return

        bounded = Twist()
        bounded.linear.x = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        bounded.angular.z = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        self.last_cmd = bounded
        self.last_cmd_time = self.get_clock().now()
        self.timed_out = False
        self.pub.publish(bounded)

    def timer_callback(self):
        now = self.get_clock().now()
        if self.last_cmd_time is None:
            self.pub.publish(Twist())
            return
        elapsed = (now - self.last_cmd_time).nanoseconds * 1e-9
        if elapsed > self.timeout_sec:
            if not self.timed_out:
                self.get_logger().info(f'Stale cmd_vel ({elapsed:.3f}s > {self.timeout_sec:.3f}s); stopping robot.')
                self.timed_out = True
            self.pub.publish(Twist())
        else:
            self.pub.publish(self.last_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
