#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from gazebo_msgs.srv import SetEntityState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

class DynamicObstacleManager(Node):
    def __init__(self):
        super().__init__('dynamic_obstacle_manager', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        for k, v in [('a1_p0',[0.5,-2.8]),('a1_p1',[0.5,2.8]),('a1_v',0.35),('a2_p0',[3.0,0.5]),('a2_p1',[-3.0,0.5]),('a2_v',0.30)]:
            self.declare_parameter(k, v)
        self.cli = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.pub = self.create_publisher(Float64MultiArray, '/dynamic_obstacles_state', 10)
        self.create_service(Trigger, '/reset_dynamic_agents', self._reset_cb)
        self.t0 = None
        self.create_timer(0.04, self._step)

    def _reset_cb(self, req, res):
        self.t0 = self.get_clock().now().nanoseconds * 1e-9
        res.success = True; res.message = 'reset'
        return res
    def _eval_pingpong(self, p0, p1, spd, t):
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        dist = math.hypot(dx, dy)
        if dist < 1e-6 or spd <= 0.0: return p0[0], p0[1], 0.0, 0.0
        T = dist / spd
        phase = (t % (2.0 * T)) / T
        u, sgn = (phase, 1.0) if phase <= 1.0 else (2.0 - phase, -1.0)
        return p0[0] + u * dx, p0[1] + u * dy, sgn * spd * dx / dist, sgn * spd * dy / dist
    def _step(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0 or not self.cli.service_is_ready(): return
        if self.t0 is None: self.t0 = now
        dt = max(0.0, now - self.t0)
        s1 = self._eval_pingpong(self.get_parameter('a1_p0').value, self.get_parameter('a1_p1').value, float(self.get_parameter('a1_v').value), dt)
        s2 = self._eval_pingpong(self.get_parameter('a2_p0').value, self.get_parameter('a2_p1').value, float(self.get_parameter('a2_v').value), dt)
        for name, st in [('dyn_agent_1', s1), ('dyn_agent_2', s2)]:
            r = SetEntityState.Request(); r.state.name = name
            r.state.pose.position.x, r.state.pose.position.y, r.state.pose.position.z = st[0], st[1], 0.6
            r.state.pose.orientation.w = 1.0; self.cli.call_async(r)
        msg = Float64MultiArray(); msg.data = [*s1, 0.25, *s2, 0.25]; self.pub.publish(msg)

def main():
    rclpy.init()
    node = DynamicObstacleManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
