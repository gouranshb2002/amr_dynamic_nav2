#!/usr/bin/env python3
import argparse, csv, json, math, os, signal, subprocess, sys, time, yaml
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.msg import ContactsState
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from amr_benchmark.evaluator import EpisodeEvaluator, wrap_angle, yaw_from_quat

def make_pose(frame, stamp, x, y, yaw):
    p = PoseStamped()
    p.header.frame_id, p.header.stamp = frame, stamp
    p.pose.position.x, p.pose.position.y = float(x), float(y)
    p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw * 0.5), math.cos(yaw * 0.5)
    return p

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', type=int, default=20)
    ap.add_argument('--out-dir', default='results/phase2_static_pilot')
    ap.add_argument('--gui', action='store_true')
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    os.system("pkill -9 -f 'phase2_nav2|gzserver|gzclient|amcl|bt_navigator|controller_server' 2>/dev/null || true")
    time.sleep(1.0)
    proc = subprocess.Popen(['ros2', 'launch', 'amr_bringup', 'phase2_nav2.launch.py', f"gui:={'true' if args.gui else 'false'}"],
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, preexec_fn=os.setsid)

    rclpy.init()
    nav = BasicNavigator()
    pkg_sim = get_package_share_directory('amr_simulation')
    pkg_bm = get_package_share_directory('amr_benchmark')
    ev = EpisodeEvaluator(os.path.join(pkg_sim, 'maps', 'dev_warehouse_obstacles.json'))
    with open(os.path.join(pkg_bm, 'config', 'static_pilot_routes.yaml')) as f:
        routes = yaml.safe_load(f)['routes'][:args.episodes]

    gt = [None]
    nav.create_subscription(Odometry, '/ground_truth/odom', lambda m: gt.__setitem__(0, m), 10)
    nav.create_subscription(ContactsState, '/bumper_states', lambda m: ev.add_contacts(len(m.states)), 10)
    cmd_pub = nav.create_publisher(Twist, '/cmd_vel', 10)
    state_cli = nav.create_client(SetEntityState, '/gazebo/set_entity_state')
    records = []

    try:
        nav.waitUntilNav2Active()
        assert state_cli.wait_for_service(timeout_sec=10.0)
        for r in routes:
            sx, sy, syaw = r['start']
            gx, gy, gyaw = r['goal']
            cmd_pub.publish(Twist())
            req = SetEntityState.Request()
            req.state.name = 'amr_bot'
            req.state.pose.position.x, req.state.pose.position.y = float(sx), float(sy)
            req.state.pose.orientation.z, req.state.pose.orientation.w = math.sin(syaw * 0.5), math.cos(syaw * 0.5)
            fut = state_cli.call_async(req)
            while not fut.done(): rclpy.spin_once(nav, timeout_sec=0.02)
            nav.setInitialPose(make_pose('map', nav.get_clock().now().to_msg(), sx, sy, syaw))
            t_settle = nav.get_clock().now().nanoseconds + int(1.2 * 1e9)
            while nav.get_clock().now().nanoseconds < t_settle: rclpy.spin_once(nav, timeout_sec=0.05)
            nav.clearAllCostmaps()
            ev.reset()
            w0, s0 = time.monotonic(), nav.get_clock().now().nanoseconds * 1e-9
            nav.goToPose(make_pose('map', nav.get_clock().now().to_msg(), gx, gy, gyaw))
            term = 'running'
            while not nav.isTaskComplete():
                rclpy.spin_once(nav, timeout_sec=0.05)
                if gt[0]:
                    p = gt[0].pose.pose
                    ev.update_pose(p.position.x, p.position.y, yaw_from_quat(p.orientation))
                if ev.bumper_contacts > 0 or ev.min_clearance <= 0.0:
                    term = 'collision'; nav.cancelTask(); break
                if (nav.get_clock().now().nanoseconds * 1e-9 - s0) >= 120.0:
                    term = 'timeout'; nav.cancelTask(); break
            res = nav.getResult()
            p = gt[0].pose.pose
            ev.update_pose(p.position.x, p.position.y, yaw_from_quat(p.orientation))
            pos_err = math.hypot(p.position.x - gx, p.position.y - gy)
            yaw_err = abs(wrap_angle(yaw_from_quat(p.orientation) - gyaw))
            sim_dt = max(1e-3, nav.get_clock().now().nanoseconds * 1e-9 - s0)
            wall_dt = max(1e-3, time.monotonic() - w0)
            if term == 'running':
                term = 'success' if (res == TaskResult.SUCCEEDED and pos_err <= 0.15 and yaw_err <= 0.15 and ev.bumper_contacts == 0) else 'nav2_failed'
            rec = {'episode_id': r['id'], 'start': r['start'], 'goal': r['goal'], 'termination': term,
                   'pos_err_m': round(pos_err, 4), 'yaw_err_rad': round(yaw_err, 4),
                   'min_clearance_m': round(ev.min_clearance, 4), 'bumper_contacts': ev.bumper_contacts,
                   'path_length_m': round(ev.path_length, 3), 'sim_time_s': round(sim_dt, 2), 'rtf': round(sim_dt / wall_dt, 2)}
            records.append(rec)
            print(f"Ep {r['id']:02d}: {term:11s} | pos_err={pos_err:.3f}m yaw_err={yaw_err:.3f}rad | clear={ev.min_clearance:.3f}m | t={sim_dt:.1f}s (RTF={sim_dt/wall_dt:.2f})")
    finally:
        nav.destroyNode()
        if rclpy.ok(): rclpy.shutdown()
        os.killpg(os.getpgid(proc.pid), signal.SIGINT); proc.wait(timeout=10)

    succ = sum(1 for x in records if x['termination'] == 'success')
    cols = sum(1 for x in records if x['termination'] == 'collision' or x['bumper_contacts'] > 0)
    git_sha = subprocess.check_output(['git', 'rev-parse', '--short', 'HEAD'], text=True).strip()
    summary = {'git_commit': git_sha, 'total_episodes': len(records), 'successes': succ,
               'success_rate': round(succ / max(1, len(records)), 3), 'collisions': cols}
    with open(os.path.join(args.out_dir, 'summary.json'), 'w') as f: json.dump({'summary': summary, 'episodes': records}, f, indent=2)
    with open(os.path.join(args.out_dir, 'episodes.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(records[0].keys())); w.writeheader(); w.writerows(records)
    print(f"\n=== PILOT SUMMARY: {succ}/{len(records)} ({summary['success_rate']*100:.1f}%), collisions={cols} ===")
    sys.exit(0 if (len(records) > 0 and succ / len(records) >= 0.90 and cols == 0) else 1)

if __name__ == '__main__':
    main()
