#!/usr/bin/env python3
import math, os, signal, subprocess, sys, time, threading
import rclpy
from gazebo_msgs.msg import ContactsState
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

os.system("pkill -9 -f 'phase2_nav2|gzserver|gzclient|amcl|bt_navigator|controller_server|planner_server' 2>/dev/null || true")
time.sleep(1.0)
gui_arg = 'gui:=true' if 'gui' in sys.argv else 'gui:=false'
proc = subprocess.Popen(['ros2', 'launch', 'amr_bringup', 'phase2_nav2.launch.py', gui_arg],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, preexec_fn=os.setsid)
threading.Thread(target=lambda: [print(l.rstrip()) for l in proc.stdout if 'ERROR' in l or 'Traceback' in l], daemon=True).start()

rclpy.init()
nav = BasicNavigator()
gt, contacts = None, 0
nav.create_subscription(Odometry, '/ground_truth/odom', lambda m: globals().update(gt=m), 10)
nav.create_subscription(ContactsState, '/bumper_states', lambda m: globals().update(contacts=contacts + len(m.states)), 10)

try:
    nav.waitUntilNav2Active()
    g = PoseStamped()
    g.header.frame_id = 'map'
    g.pose.position.x, g.pose.orientation.w = 3.5, 1.0
    nav.goToPose(g)
    t0 = 0.0
    while not nav.isTaskComplete():
        rclpy.spin_once(nav, timeout_sec=0.1)
        if gt and time.monotonic() - t0 > 3.0:
            p = gt.pose.pose.position
            print(f"Progress: x={p.x:.2f}m, y={p.y:.2f}m, contacts={contacts}")
            t0 = time.monotonic()
    res = nav.getResult()
    gx = gt.pose.pose.position.x if gt else -99.0
    gy = gt.pose.pose.position.y if gt else -99.0
    err = math.hypot(gx - 3.5, gy)
    print(f"Nav2 Result={res.name}, GT=({gx:.3f}, {gy:.3f}), err={err:.3f}m, bumper_contacts={contacts}")
    sys.exit(0 if (res == TaskResult.SUCCEEDED and err < 0.15 and contacts == 0) else 1)
finally:
    nav.destroyNode()
    if rclpy.ok():
        rclpy.shutdown()
    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    proc.wait(timeout=10)
