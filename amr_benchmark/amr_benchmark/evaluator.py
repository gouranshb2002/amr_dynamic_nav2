import json, math

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

def point_to_box_dist(px, py, cx, cy, sx, sy):
    dx = max(abs(px - cx) - sx * 0.5, 0.0)
    dy = max(abs(py - cy) - sy * 0.5, 0.0)
    return math.hypot(dx, dy)

class EpisodeEvaluator:
    def __init__(self, obstacles_json_path):
        with open(obstacles_json_path, 'r') as f:
            self.obstacles = json.load(f)
        self.hx, self.hy = 0.225, 0.220
        self.reset()

    def reset(self):
        self.min_clearance = float('inf')
        self.bumper_contacts = 0
        self.path_length = 0.0
        self.prev_xy = None

    def update_pose(self, x, y, yaw):
        if self.prev_xy is not None:
            self.path_length += math.hypot(x - self.prev_xy[0], y - self.prev_xy[1])
        self.prev_xy = (x, y)
        c, s = math.cos(yaw), math.sin(yaw)
        pts = [(lx, ly) for lx in (-self.hx, 0.0, self.hx) for ly in (-self.hy, 0.0, self.hy)]
        for lx, ly in pts:
            wx, wy = x + c * lx - s * ly, y + s * lx + c * ly
            for ob in self.obstacles:
                d = point_to_box_dist(wx, wy, ob['cx'], ob['cy'], ob['sx'], ob['sy'])
                if d < self.min_clearance:
                    self.min_clearance = d

    def add_contacts(self, n):
        self.bumper_contacts += n
