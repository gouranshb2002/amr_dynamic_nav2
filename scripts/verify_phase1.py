#!/usr/bin/env python3
import math, os, signal, subprocess, sys, time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
import tf2_ros

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

class Phase1Verifier(Node):
    def __init__(self):
        super().__init__('phase1_verifier', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.scan = self.odom = self.gt = self.safe_cmd = None
        self.tf_pubs = {}
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(Odometry, '/ground_truth/odom', lambda m: setattr(self, 'gt', m), 10)
        self.create_subscription(Twist, '/cmd_vel_safe', lambda m: setattr(self, 'safe_cmd', m), 10)
        self.create_subscription(TFMessage, '/tf', self._tf_cb, 50)
        self.create_subscription(TFMessage, '/tf_static', self._tf_static_cb, 50)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

    def _tf_cb(self, msg):
        for t in msg.transforms:
            self.tf_pubs.setdefault((t.header.frame_id, t.child_frame_id), set()).add('dyn')

    def _tf_static_cb(self, msg):
        for t in msg.transforms:
            self.tf_pubs.setdefault((t.header.frame_id, t.child_frame_id), set()).add('stat')

    def spin_sim(self, sec, cmd=None):
        end_ns = self.get_clock().now().nanoseconds + int(sec * 1e9)
        while self.get_clock().now().nanoseconds < end_ns:
            if cmd is not None:
                self.cmd_pub.publish(cmd)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_topics(self, timeout=25.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.scan and self.odom and self.gt and self.get_clock().now().nanoseconds > 0:
                return True
        return False

    def reset_robot(self):
        assert self.state_cli.wait_for_service(timeout_sec=5.0), '/gazebo/set_entity_state unavailable'
        req = SetEntityState.Request()
        req.state.name = 'amr_bot'
        req.state.pose.position.z = 0.0
        req.state.pose.orientation.w = 1.0
        fut = self.state_cli.call_async(req)
        while not fut.done():
            rclpy.spin_once(self, timeout_sec=0.02)
        self.cmd_pub.publish(Twist())
        self.spin_sim(0.8)

def main():
    proc = subprocess.Popen(['ros2', 'launch', 'amr_bringup', 'phase1_bringup.launch.py', 'gui:=false'],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)
    rclpy.init()
    node = Phase1Verifier()
    res = {}
    try:
        assert node.wait_topics(25.0), 'Timed out waiting for topics'
        node.spin_sim(1.0)
        tf_ok = node.tf_buf.can_transform('odom', 'base_scan', rclpy.time.Time())
        tf_n = len(node.get_publishers_info_by_topic('/tf'))
        tfs_n = len(node.get_publishers_info_by_topic('/tf_static'))
        ov = [k for k, v in node.tf_pubs.items() if len(v) > 1]
        res['1_tf_valid_and_unique'] = (tf_ok and tf_n >= 1 and tfs_n == 1 and not ov, f'tf={tf_ok}, pubs={tf_n}/{tfs_n}, ov={ov}')

        s = node.scan
        fd = s.ranges[len(s.ranges) // 2]
        no_nan = all(not math.isnan(r) for r in s.ranges)
        res['2_lidar_alignment'] = (s.header.frame_id == 'base_scan' and no_nan and abs(fd - 1.85) < 0.05, f'dist={fd:.3f}m (exp 1.850)')

        c = Twist(); c.linear.x = 0.80
        node.cmd_pub.publish(c); node.spin_sim(0.1)
        v1 = node.safe_cmd.linear.x if node.safe_cmd else -1.0
        node.spin_sim(0.5)
        v2 = node.safe_cmd.linear.x if node.safe_cmd else -1.0
        res['3_watchdog_clamp_and_stale_stop'] = (abs(v1 - 0.20) < 1e-4 and abs(v2) < 1e-4, f'clamped={v1:.3f}, stopped={v2:.3f}')

        node.reset_robot()
        x0_g, x0_o = node.gt.pose.pose.position.x, node.odom.pose.pose.position.x
        c = Twist(); c.linear.x = 0.20
        node.spin_sim(5.0, cmd=c); node.cmd_pub.publish(Twist()); node.spin_sim(0.5)
        dx_g = node.gt.pose.pose.position.x - x0_g
        dx_o = node.odom.pose.pose.position.x - x0_o
        sd = node.scan.ranges[len(node.scan.ranges) // 2]

        y0 = yaw_from_quat(node.gt.pose.pose.orientation)
        r = Twist(); r.angular.z = 0.50
        node.spin_sim(math.pi, cmd=r); node.cmd_pub.publish(Twist()); node.spin_sim(0.5)
        dy = yaw_from_quat(node.gt.pose.pose.orientation) - y0
        res['4_straight_and_rotation_units'] = (
            abs(dx_g - 1.00) < 0.08 and abs(dx_o - 1.00) < 0.08 and abs(sd - 0.85) < 0.06 and abs(dy - math.pi / 2) < 0.18,
            f'dx_gt={dx_g:.3f}m, dx_odom={dx_o:.3f}m, scan={sd:.3f}m, dyaw={dy:.3f}rad'
        )

        rok, notes = True, []
        for i in range(3):
            node.reset_robot()
            gx, gy = node.gt.pose.pose.position.x, node.gt.pose.pose.position.y
            gyaw = yaw_from_quat(node.gt.pose.pose.orientation)
            sd = node.scan.ranges[len(node.scan.ranges) // 2]
            rok = rok and abs(gx) < 0.03 and abs(gy) < 0.03 and abs(gyaw) < 0.05 and abs(sd - 1.85) < 0.05
            notes.append(f'r{i+1}:({gx:.2f},{gy:.2f},{gyaw:.2f},s={sd:.2f})')
        res['5_repeat_reset'] = (rok, ' '.join(notes))
    finally:
        node.destroy_node(); rclpy.shutdown()
        os.killpg(os.getpgid(proc.pid), signal.SIGINT); proc.wait(timeout=10)

    print('\n=== PHASE 1 EXIT CHECK REPORT ===')
    ok_all = True
    for k, (ok, msg) in res.items():
        ok_all = ok_all and ok
        print(f'[{"PASS" if ok else "FAIL"}] {k}: {msg}')
    sys.exit(0 if ok_all else 1)

if __name__ == '__main__':
    main()
