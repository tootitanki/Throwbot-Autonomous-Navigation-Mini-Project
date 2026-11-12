"""Sends the goal and logs ground-truth based metrics for one trial.

Ground truth: robot pose (TF map->base_footprint; map frame origin = world (-4.5, 0)), moving-box position (Gazebo joint state).
Detection: a cell with cost 100 (lethal) in /local_costmap/costmap inside the box's CURRENT bounding box (+0.10 m).
Visibility: geometric only - the box is inside the sensor's field of view and range and the sensor->box-centre line is not blocked
by a fixed box or wall. It does not model reflectivity, specular loss or depth-camera artefacts.
All times are simulation time."""
import json, math, os, sys, time
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory as share
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan, PointCloud2
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import Buffer, TransformListener

SHIFT = 4.5                      # map x = world x + 4.5 ; map y = world y
OBS_X = 0.5 + SHIFT              # crossing line in map frame
BODY_L, BODY_W = 0.20, 0.15      # body half length/width (0.40 x 0.30 m)
CONTACT_TOL = 0.01              # body-box gap <= 1 cm counts as contact
OBS_HALF, ROBOT_R = 0.25, 0.25   # moving box half size; conservative circular robot footprint (circumscribes 0.40 x 0.30 body)
BOXES = [(-2.5, 1.5, 1.0, 1.0), (-1.0, -2.0, 1.2, 0.8), (3.0, 2.0, 1.0, 1.5), (3.0, -2.5, 1.5, 1.0)]
STATIC_RECTS = [(cx + SHIFT, cy, sx / 2, sy / 2) for cx, cy, sx, sy in BOXES]   # map-frame centre + half sizes
ROOM = (-6.0 + SHIFT, 6.0 + SHIFT, -4.0, 4.0)                                      # inner room bounds, map frame
HAZARD_Y = 0.85                  # box centre within this lateral distance of the route (y=0) counts as "in the corridor"
DET_MARGIN = 0.10


def rect_dist(px, py, cx, cy, hx, hy):
    return np.hypot(np.maximum(np.abs(px - cx) - hx, 0), np.maximum(np.abs(py - cy) - hy, 0))


def seg_hits_rect(x0, y0, x1, y1, cx, cy, hx, hy):
    t0, t1 = 0.0, 1.0
    for a0, d, lo, hi in ((x0, x1 - x0, cx - hx, cx + hx), (y0, y1 - y0, cy - hy, cy + hy)):
        if abs(d) < 1e-9:
            if a0 < lo or a0 > hi: return False
        else:
            ta, tb = (lo - a0) / d, (hi - a0) / d
            if ta > tb: ta, tb = tb, ta
            t0, t1 = max(t0, ta), min(t1, tb)
            if t0 > t1: return False
    return True


class Study(Node):
    def __init__(self):
        super().__init__('study_logger')
        for k, v in (('sensor', 'lidar'), ('speed', 0.5), ('seed', 0), ('out_dir', '/tmp/results'), ('run_id', 'manual'),
                     ('goal_x', 9.0), ('goal_y', 0.0), ('timeout', 150.0)):
            self.declare_parameter(k, v)
        self.sensor = self.get_parameter('sensor').value
        cfg = yaml.safe_load(open(os.path.join(share('throwbot_sensors'), 'config', 'sensors.yaml')))[self.sensor]
        self.mx = cfg['mount']['x']; self.rmin = cfg['range_min']
        # the sensor can see to range_max, but the costmap only MARKS obstacles within obstacle_max_range (ultrasonic: no extra limit)
        self.rmax = min(cfg['range_max'], cfg['costmap'].get('obstacle_max_range', cfg['range_max']))
        self.half_fov = cfg['hfov'] / 2 if cfg['kind'] == 'depth' else (cfg['max_angle'] - cfg['min_angle']) / 2
        self.tfb = Buffer(); TransformListener(self.tfb, self)
        self.oy = None; self.pose = None; self.yaw = 0.0; self.last = None; self.path = 0.0
        self.min_clear = float('inf'); self.t_min = None; self.t_start = None; self.t_move = None
        self.t_haz = None; self.t_det = None; self.gap_det = None; self.first_vis_move = None
        self.vis_n = 0; self.det_vis_n = 0; self.phantom = []; self.stopped = 0.0
        self.trace = []; self.prev_oy = None
        self.create_subscription(JointState, '/obstacle/joint_state', self.on_obs, 10)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.on_cm, 10)
        self.n_sensor = 0; self.n_cm = 0; self.counting = False
        if cfg['kind'] == 'depth':
            self.create_subscription(PointCloud2, '/sensor/depth/points', self.on_sensor, qos_profile_sensor_data)
        else:
            self.create_subscription(LaserScan, '/sensor/scan', self.on_sensor, qos_profile_sensor_data)
        self.cli = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.create_timer(0.1, self.sample)

    def on_sensor(self, m):
        if self.counting: self.n_sensor += 1

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_obs(self, m):
        if 'slide' in m.name:
            self.oy = -3.0 + m.position[m.name.index('slide')]
            if self.prev_oy is not None and abs(self.oy - self.prev_oy) > 1e-3 and self.t_move is None:
                self.t_move = self.now()
            self.prev_oy = self.oy

    def visible(self):
        """box inside FOV and range of the sensor, centre line not blocked by a fixed object"""
        if self.pose is None or self.oy is None: return False
        x, y = self.pose
        sx, sy = x + self.mx * math.cos(self.yaw), y + self.mx * math.sin(self.yaw)
        ok = False
        for dx in (-OBS_HALF, 0, OBS_HALF):
            for dy in (-OBS_HALF, 0, OBS_HALF):
                px, py = OBS_X + dx, self.oy + dy
                d = math.hypot(px - sx, py - sy)
                b = (math.atan2(py - sy, px - sx) - self.yaw + math.pi) % (2 * math.pi) - math.pi
                if self.rmin <= d <= self.rmax and abs(b) <= self.half_fov:
                    ok = True; break
            if ok: break
        if not ok: return False
        for cx, cy, hx, hy in STATIC_RECTS:
            if seg_hits_rect(sx, sy, OBS_X, self.oy, cx, cy, hx, hy): return False
        return True

    def clearance(self):
        """Gap between the true oriented 0.40x0.30 body and the 0.5x0.5 box (m). 0 = touching; physics prevents
        penetration, so <=CONTACT_TOL means contact. Pose is Gazebo's world pose (OdometryPublisher), not wheel odometry."""
        x, y = self.pose
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        best = 1e9
        corners = [(BODY_L, BODY_W), (BODY_L, -BODY_W), (-BODY_L, -BODY_W), (-BODY_L, BODY_W)]
        for i in range(4):
            (a, b), (a2, b2) = corners[i], corners[(i + 1) % 4]
            for k in range(21):
                u = k / 20
                px, py = a + (a2 - a) * u, b + (b2 - b) * u
                wx, wy = x + c * px - s * py, y + s * px + c * py
                best = min(best, float(rect_dist(wx, wy, OBS_X, self.oy, OBS_HALF, OBS_HALF)))
        return best

    def sample(self):
        try:
            t = self.tfb.lookup_transform('map', 'base_footprint', rclpy.time.Time())
        except Exception:
            return
        x, y = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        self.yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (x, y)
        now = self.now()
        if self.last:
            d = math.hypot(x - self.last[0], y - self.last[1])
            self.path += d
            if self.t_start is not None and d / 0.1 < 0.02 and self.t_move is not None: self.stopped += 0.1
        self.last = (x, y)
        if self.oy is None: return
        c = self.clearance()
        if c < self.min_clear: self.min_clear, self.t_min = c, now
        vis = self.visible()
        if self.t_haz is None and abs(self.oy) < HAZARD_Y and self.t_move is not None and vis:
            self.t_haz = now
        self.trace.append((round(now, 2), round(x, 3), round(y, 3), round(self.yaw, 3), round(self.oy, 3), round(c, 3), int(vis)))

    def on_cm(self, m):
        if self.counting: self.n_cm += 1
        if self.pose is None or self.oy is None or self.t_start is None: return
        info = m.info
        a = np.array(m.data, dtype=np.int16).reshape(info.height, info.width)
        iy, ix = np.nonzero(a >= 100)
        # the local costmap lives in the 'odom' frame (= Gazebo world); the logger works in the map frame (map x = odom x + SHIFT)
        off = SHIFT if m.header.frame_id == 'odom' else 0.0
        px = info.origin.position.x + off + (ix + 0.5) * info.resolution
        py = info.origin.position.y + (iy + 0.5) * info.resolution
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        in_box = rect_dist(px, py, OBS_X, self.oy, OBS_HALF + DET_MARGIN, OBS_HALF + DET_MARGIN) <= 0 if len(px) else np.array([])
        detected = bool(np.any(in_box)) if len(px) else False
        vis = self.visible()
        if self.t_move is not None and vis:
            self.vis_n += 1; self.det_vis_n += int(detected)
        if self.t_haz is not None and self.t_det is None and detected and stamp >= self.t_haz:
            self.t_det = stamp
            self.gap_det = max(float(rect_dist(self.pose[0], self.pose[1], OBS_X, self.oy, OBS_HALF, OBS_HALF)), 0.0)   # robot centre to box surface
        # phantom cells: lethal cells farther than 0.2 m from every true obstacle (walls, fixed boxes, moving box)
        if len(px):
            d = rect_dist(px, py, OBS_X, self.oy, OBS_HALF, OBS_HALF)
            for cx, cy, hx, hy in STATIC_RECTS:
                d = np.minimum(d, rect_dist(px, py, cx, cy, hx, hy))
            wall = np.minimum.reduce([np.abs(px - ROOM[0]), np.abs(ROOM[1] - px), np.abs(py - ROOM[2]), np.abs(ROOM[3] - py)])
            d = np.minimum(d, wall)
            self.phantom.append(int(np.sum(d > 0.2)))
        else:
            self.phantom.append(0)

    def run(self):
        wall0 = time.time()
        self.get_logger().info('waiting for navigate_to_pose and TF ...')
        while not self.cli.wait_for_server(timeout_sec=2.0): rclpy.spin_once(self, timeout_sec=0.1)
        while not self.tfb.can_transform('map', 'base_footprint', rclpy.time.Time()): rclpy.spin_once(self, timeout_sec=0.2)
        t_end = time.time() + 5.0
        while time.time() < t_end: rclpy.spin_once(self, timeout_sec=0.1)   # let costmaps and sensors settle
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = self.get_parameter('goal_x').value
        goal.pose.pose.position.y = self.get_parameter('goal_y').value
        goal.pose.pose.orientation.w = 1.0
        status = -1; accepted = False
        for attempt in range(1, 6):
            fut = self.cli.send_goal_async(goal); rclpy.spin_until_future_complete(self, fut)
            h = fut.result()
            if not h.accepted:
                time.sleep(2.0); continue
            accepted = True
            self.t_start = self.now(); self.wall_start = time.time(); self.counting = True
            res = h.get_result_async()
            end = time.time() + self.get_parameter('timeout').value
            while not res.done() and time.time() < end: rclpy.spin_once(self, timeout_sec=0.05)
            if not res.done():
                h.cancel_goal_async(); status = -2; break
            status = res.result().status
            break
        t_fin = self.now(); self.counting = False; self.wall_run = time.time() - getattr(self, 'wall_start', time.time())
        for _ in range(10): rclpy.spin_once(self, timeout_sec=0.05)
        out = self.summary(status, accepted, t_fin, time.time() - wall0)
        d = self.get_parameter('out_dir').value
        os.makedirs(os.path.join(d, 'traces'), exist_ok=True)
        rid = self.get_parameter('run_id').value
        with open(os.path.join(d, 'traces', f'{rid}.csv'), 'w') as f:
            f.write('t_sim,robot_x,robot_y,robot_yaw,box_y,clearance,box_visible\n')
            for r in self.trace: f.write(','.join(map(str, r)) + '\n')
        with open(os.path.join(d, 'summary.jsonl'), 'a') as f: f.write(json.dumps(out) + '\n')
        self.get_logger().info('STUDY_RESULT ' + json.dumps(out))
        return 0

    def summary(self, status, accepted, t_fin, wall):
        ts = self.t_start or 0.0
        det_before_min = self.t_det is not None and self.t_min is not None and self.t_det <= self.t_min
        return {
            'run_id': self.get_parameter('run_id').value, 'sensor': self.sensor, 'speed': self.get_parameter('speed').value,
            'seed': self.get_parameter('seed').value,
            'goal_status': 'SUCCEEDED' if status == 4 else ('TIMEOUT' if status == -2 else f'status_{status}'),
            'goal_accepted': accepted, 'duration_sim_s': round(t_fin - ts, 2), 'path_len_m': round(self.path, 2),
            'min_clearance_m': round(self.min_clear, 3), 'overlap': bool(self.min_clear <= CONTACT_TOL), 'contact': bool(self.min_clear <= CONTACT_TOL), 'near_contact_5cm': bool(self.min_clear <= 0.05),
                        't_motion_start_s': None if self.t_move is None else round(self.t_move - ts, 2),
            't_in_corridor_visible_s': None if self.t_haz is None else round(self.t_haz - ts, 2),
            't_detected_in_corridor_s': None if self.t_det is None else round(self.t_det - ts, 2),
            'detect_lag_s': None if (self.t_det is None or self.t_haz is None) else round(self.t_det - self.t_haz, 2),
            'detect_gap_m': None if self.gap_det is None else round(self.gap_det, 2),
            'detected_before_closest_approach': bool(det_before_min),
            'lead_time_s': None if (self.t_det is None or self.t_min is None) else round(self.t_min - self.t_det, 2),
            'visible_frames': self.vis_n, 'tracking_recall': None if not self.vis_n else round(self.det_vis_n / self.vis_n, 3),
            'phantom_cells_mean': round(float(np.mean(self.phantom)), 1) if self.phantom else None,
            'phantom_cells_max': int(max(self.phantom)) if self.phantom else None,
            'wall_s': round(wall, 1),
            'sensor_rate_sim_hz': round(self.n_sensor / max(t_fin - ts, 1e-6), 2),
            'sensor_rate_wall_hz': round(self.n_sensor / max(self.wall_run, 1e-6), 2),
            'costmap_rate_sim_hz': round(self.n_cm / max(t_fin - ts, 1e-6), 2),
            'real_time_factor': round((t_fin - ts) / max(self.wall_run, 1e-6), 3)}


def main():
    rclpy.init(); n = Study(); code = n.run(); n.destroy_node(); rclpy.shutdown(); sys.exit(code)
