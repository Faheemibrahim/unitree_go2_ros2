#!/usr/bin/env python3
"""Localization guard for AMCL on the real Go2: keeps map -> odom sane where the room no longer looks like the map
(objects added since mapping, e.g. cardboard on the floor), and re-seeds AMCL itself when it has wandered off.

Point-LIO odometry drifts slowly: on two lab walks (2026-10-17) the true map -> odom moved at most 0.45 m / 3.4 deg in
total, p95 3.5 cm/s and 0.16 deg/s, never faster than 8.3 cm/s. AMCL going wrong moves it by metres. So:
  follow   AMCL's map -> odom is followed, but no faster than follow_v / follow_w ("drift budget"). For settle_s after a
           pose is given (launch start pose, RViz "2D Pose Estimate") the budget is settle_v / settle_w: a rough
           start pose still gets corrected quickly, while later AMCL can only pull the pose slowly.
  diverged AMCL further than diverge_dist / diverge_yaw from the held pose H is not followed; after reseed_after_s,
           H is sent to AMCL on /initialpose (what "2D Pose Estimate" in RViz does by hand).
  big corrections (wrong start pose, "kidnapped") are off by default: in the lab a wrong pose often matches the map as
           well as the right one (0.5-0.75 averaged, cardboard walk 2026-10-17), so a better score is no proof. Then
           only "2D Pose Estimate" moves H by more than the drift budget. big_corrections:=strict adopts AMCL only when
           it has held one map -> odom (within adopt_stable_m / adopt_stable_deg) for adopt_window_s while matching the
           map clearly better (mean >= adopt_score and >= H's + adopt_gain); while it is on its way there the guard
           waits instead of re-seeding. Strict also searches when lost: +-search_xy / +-search_yaw around H; a result
           that matches well, clearly better, with no close rival, and found again by the next search, is adopted.
Score = share of scan points within hit_dist of a map wall (one scan is noisy: right vs 0.3 m off is 0.84 vs 0.55
median on a clean walk, so the guard decides on window means and never on a single scan).
mode: passive = score and log only; reseed = AMCL publishes map -> odom, the guard only re-seeds it;
      full = the guard publishes map -> odom (AMCL's tf_broadcast must be false; real_navigation.launch.py does that).
A "2D Pose Estimate" from RViz resets H too.
Defaults = the best of a one-factor sweep on replays of cardboard_walk_1 (cardboard not in the map), 2 runs each,
scored against the Point-LIO path: drift budget 0.02 (0.05 / 0.10 worse), settle 20 s at 0.10 (10 s slightly worse),
diverge 0.6 m / 8 deg (0.4 / 5 and 1.0 / 15 worse), re-seed after 3 s (1 s and 6 s worse). Start pose must be within
about 0.5 m / 10 deg (1 m / 20 deg is not recovered).
  ros2 run unitree_go2_real localization_guard.py --ros-args -p mode:=full"""
import collections
import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from scipy import ndimage
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], a[2] + b[2])


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c * a[0] - s * a[1], s * a[0] - c * a[1], -a[2])


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def apart(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]), abs(wrap(a[2] - b[2]))


class Guard(Node):
    def __init__(self):
        super().__init__('localization_guard')
        p = {'mode': 'full', 'scan_topic': 'scan', 'map_frame': 'map', 'odom_frame': 'odom',
             'base_frame': 'base_footprint', 'hit_dist': 0.10, 'min_range': 0.8, 'max_range': 12.0, 'max_points': 250,
             'follow_v': 0.02, 'follow_w_deg': 0.2, 'settle_s': 20.0, 'settle_v': 0.10, 'settle_w_deg': 1.0,
             'diverge_dist': 0.6, 'diverge_yaw_deg': 8.0, 'window_s': 3.0,
             'big_corrections': 'off', 'adopt_window_s': 10.0, 'adopt_stable_m': 0.15, 'adopt_stable_deg': 3.0,
             'adopt_score': 0.65, 'adopt_gain': 0.20, 'reseed_after_s': 3.0, 'reseed_cooldown_s': 3.0,
             'lost_score': 0.35, 'search_xy': 1.2, 'search_yaw_deg': 25.0, 'search_every_s': 1.0,
             'search_score': 0.50, 'search_gain': 0.20, 'rival_gap': 0.08, 'tf_rate': 20.0, 'tf_future_s': 0.2,
             'log_file': ''}
        for k, v in p.items():
            self.declare_parameter(k, v)
        self.c = {k: self.get_parameter(k).value for k in p}
        assert self.c['mode'] in ('passive', 'reseed', 'full'), self.c['mode']
        assert self.c['big_corrections'] in ('off', 'strict'), self.c['big_corrections']
        self.buf = Buffer()
        TransformListener(self.buf, self, spin_thread=True)
        self.tfb = TransformBroadcaster(self)
        self.map = None
        self.mo_amcl = None        # AMCL's map -> odom
        self.mo_held = None        # the trusted map -> odom
        self.amcl_after = 0        # ignore AMCL poses stamped before this (ns): they predate our re-seed
        self.pending = collections.deque(maxlen=20)     # scans waiting for their odometry TF
        self.window = collections.deque()               # (t, sA, sH, AMCL map -> odom) while diverged
        self.diverged_since = self.lost_since = self.last_t = self.candidate = None
        self.last_reseed = self.last_search = -1e9
        self.settle_until = -1e9   # the fast drift budget applies until then (s, ROS clock)
        self.own_stamp = None      # stamp of our last re-seed: our own /initialpose is no new start pose
        self.base_laser = {}
        self.n = {'scans': 0, 'dropped': 0, 'follow': 0, 'diverged': 0, 'adopt': 0, 'reseed': 0, 'search': 0, 'found': 0}
        self.log = open(self.c['log_file'], 'w') if self.c['log_file'] else None
        if self.log:
            self.log.write('# t sA sH dA_m dA_deg state event\n')
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self.on_map, latched)
        self.create_subscription(LaserScan, self.c['scan_topic'], lambda m: self.pending.append(m), qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self.on_amcl, latched)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self.on_initialpose, 10)
        self.pub_init = self.create_publisher(PoseWithCovarianceStamped, 'initialpose', 10)
        self.pub_state = self.create_publisher(String, '~/state', 10)
        self.create_timer(0.02, self.drain)
        if self.c['mode'] == 'full':
            self.create_timer(1.0 / self.c['tf_rate'], self.publish_tf)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f"mode {self.c['mode']}, big corrections {self.c['big_corrections']}, drift budget "
                               f"{self.c['follow_v']} m/s {self.c['follow_w_deg']} deg/s (first {self.c['settle_s']} s after a pose: "
                               f"{self.c['settle_v']} m/s {self.c['settle_w_deg']} deg/s), diverged beyond {self.c['diverge_dist']} m "
                               f"{self.c['diverge_yaw_deg']} deg, re-seed after {self.c['reseed_after_s']} s")

    # --- inputs -------------------------------------------------------------------------------------------------
    def on_map(self, m):
        a = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
        dist = ndimage.distance_transform_edt(a < 65) * m.info.resolution     # metres to the nearest wall cell
        sigma = self.c['hit_dist'] * 0.7
        self.map = dict(res=m.info.resolution, ox=m.info.origin.position.x, oy=m.info.origin.position.y,
                        w=m.info.width, h=m.info.height, dist=dist.astype(np.float32),
                        like=np.exp(-0.5 * (dist / sigma) ** 2).astype(np.float32))
        self.get_logger().info(f'map {m.info.width}x{m.info.height} @ {m.info.resolution:.3f} m')

    def odom_base(self, stamp):
        t = self.buf.lookup_transform(self.c['odom_frame'], self.c['base_frame'], stamp)
        return (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))

    def on_amcl(self, m):
        if Time.from_msg(m.header.stamp).nanoseconds < self.amcl_after:
            return
        try:
            ob = self.odom_base(Time.from_msg(m.header.stamp))
        except Exception:   # noqa: BLE001 - odometry not there yet
            return
        q = m.pose.pose
        self.mo_amcl = compose((q.position.x, q.position.y, yaw_of(q.orientation)), inverse(ob))
        if self.mo_held is None:
            self.mo_held = self.mo_amcl
            self.settle_until = self.now_s() + self.c['settle_s']

    def on_initialpose(self, m):
        """RViz "2D Pose Estimate" (or our own re-seed): that pose is the truth from now on."""
        try:
            ob = self.odom_base(Time.from_msg(m.header.stamp))
        except Exception:   # noqa: BLE001 - RViz stamps 0 or "now": use the latest odometry
            try:
                ob = self.odom_base(Time())
            except Exception:   # noqa: BLE001
                return
        q = m.pose.pose
        self.set_held(compose((q.position.x, q.position.y, yaw_of(q.orientation)), inverse(ob)))
        if (m.header.stamp.sec, m.header.stamp.nanosec) != self.own_stamp:
            self.settle_until = self.now_s() + self.c['settle_s']

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_held(self, mo):
        self.mo_held = self.mo_amcl = mo
        self.amcl_after = self.get_clock().now().nanoseconds
        self.diverged_since = self.lost_since = self.candidate = None
        self.window.clear()

    def drain(self):
        """Score queued scans whose odometry TF has arrived (never blocks; scans waiting > 0.5 s are dropped)."""
        while self.pending:
            m = self.pending[0]
            stamp = Time.from_msg(m.header.stamp)
            if self.buf.can_transform(self.c['odom_frame'], self.c['base_frame'], stamp):
                self.pending.popleft()
                self.on_scan(m, stamp)
            elif (self.get_clock().now() - stamp).nanoseconds > 5e8:
                self.pending.popleft()
                self.n['dropped'] += 1
            else:
                return

    # --- scoring ------------------------------------------------------------------------------------------------
    def points(self, m):
        frame = m.header.frame_id
        if frame not in self.base_laser:
            t = self.buf.lookup_transform(self.c['base_frame'], frame, Time())
            self.base_laser[frame] = (t.transform.translation.x, t.transform.translation.y, yaw_of(t.transform.rotation))
        r = np.asarray(m.ranges, dtype=np.float32)
        a = m.angle_min + m.angle_increment * np.arange(len(r))
        k = np.isfinite(r) & (r >= max(self.c['min_range'], m.range_min)) & (r <= min(self.c['max_range'], m.range_max))
        r, a = r[k], a[k]
        if len(r) > self.c['max_points']:
            i = np.linspace(0, len(r) - 1, self.c['max_points']).astype(int)
            r, a = r[i], a[i]
        x, y, w = self.base_laser[frame]
        c, s = math.cos(w), math.sin(w)
        px, py = r * np.cos(a), r * np.sin(a)
        return np.c_[x + c * px - s * py, y + s * px + c * py]

    def lookup(self, field, X, Y):
        M = self.map
        i = np.floor((X - M['ox']) / M['res']).astype(np.int64)
        j = np.floor((Y - M['oy']) / M['res']).astype(np.int64)
        ok = (i >= 0) & (j >= 0) & (i < M['w']) & (j < M['h'])
        out = np.full(X.shape, 9.0 if field == 'dist' else 0.0, np.float32)
        out[ok] = M[field][j[ok], i[ok]]
        return out

    def score(self, pose, P):
        c, s = math.cos(pose[2]), math.sin(pose[2])
        d = self.lookup('dist', pose[0] + c * P[:, 0] - s * P[:, 1], pose[1] + s * P[:, 0] + c * P[:, 1])
        return float(np.mean(d <= self.c['hit_dist']))

    def grid_search(self, center, P, r_xy, step_xy, r_yaw, step_yaw):
        """All poses on a grid around center -> (poses (K,3), smooth score (K,))."""
        xs = np.arange(-r_xy, r_xy + 1e-9, step_xy)
        off = np.stack(np.meshgrid(xs, xs, indexing='ij'), -1).reshape(-1, 2)
        poses, scores = [], []
        for dy in np.arange(-r_yaw, r_yaw + 1e-9, step_yaw):
            w = center[2] + dy
            c, s = math.cos(w), math.sin(w)
            Q = np.c_[center[0] + c * P[:, 0] - s * P[:, 1], center[1] + s * P[:, 0] + c * P[:, 1]]
            X, Y = Q[None, :, 0] + off[:, :1], Q[None, :, 1] + off[:, 1:]
            scores.append(self.lookup('like', X, Y).mean(1))
            poses.append(np.c_[center[0] + off, np.full(len(off), w)])
        return np.vstack(poses), np.concatenate(scores)

    def search(self, center, P):
        """Wide search around center -> (pose, score, best rival's score)."""
        R, Y = self.c['search_xy'], math.radians(self.c['search_yaw_deg'])
        poses, sc = self.grid_search(center, P, R, 0.10, Y, math.radians(2.0))
        b = int(np.argmax(sc))
        far = (np.hypot(poses[:, 0] - poses[b, 0], poses[:, 1] - poses[b, 1]) > 0.4) | \
              (np.abs(np.angle(np.exp(1j * (poses[:, 2] - poses[b, 2])))) > math.radians(8.0))
        rival_pose = poses[far][int(np.argmax(sc[far]))] if far.any() else None
        fine, fsc = self.grid_search(tuple(poses[b]), P, 0.10, 0.025, math.radians(2.0), math.radians(0.5))
        best = tuple(fine[int(np.argmax(fsc))])
        rival = self.score(tuple(rival_pose), P) if rival_pose is not None else 0.0
        return best, self.score(best, P), rival

    # --- decisions ----------------------------------------------------------------------------------------------
    def on_scan(self, m, stamp):
        if self.map is None or self.mo_held is None:
            return
        try:
            ob = self.odom_base(stamp)
            P = self.points(m)
        except Exception:   # noqa: BLE001 - laser TF not there yet
            return
        if len(P) < 30:
            return
        c, now = self.c, stamp.nanoseconds * 1e-9
        dt = min(max(now - self.last_t, 0.0), 0.5) if self.last_t is not None else 0.0
        self.last_t = now
        self.n['scans'] += 1
        H = compose(self.mo_held, ob)
        A = compose(self.mo_amcl, ob) if self.mo_amcl is not None else H
        sH, sA = self.score(H, P), self.score(A, P)
        dA, dY = apart(A, H)
        act = c['mode'] != 'passive'
        event = ''
        if dA <= c['diverge_dist'] and dY <= math.radians(c['diverge_yaw_deg']):
            state = 'follow'
            self.diverged_since = None
            if act and (dA > 0 or dY > 0):     # step H toward A within the drift budget
                settling = now < self.settle_until
                v, w = (c['settle_v'], c['settle_w_deg']) if settling else (c['follow_v'], c['follow_w_deg'])
                f = min(1.0, v * dt / dA) if dA > 1e-6 else 1.0
                step_w = max(-math.radians(w) * dt, min(math.radians(w) * dt, wrap(A[2] - H[2])))
                H = (H[0] + f * (A[0] - H[0]), H[1] + f * (A[1] - H[1]), H[2] + step_w)
                self.mo_held = compose(H, inverse(ob))
        else:
            state = 'diverged'
            if self.diverged_since is None:
                self.diverged_since = now
                self.window.clear()
            self.window.append((now, sA, sH, self.mo_amcl))
            while self.window and self.window[0][0] < now - c['adopt_window_s']:
                self.window.popleft()
            span = now - self.diverged_since
            mA = np.mean([w[1] for w in self.window])
            mH = np.mean([w[2] for w in self.window])
            strict = c['big_corrections'] == 'strict'
            stable = all(apart(w[3], self.mo_amcl)[0] <= c['adopt_stable_m'] and
                         apart(w[3], self.mo_amcl)[1] <= math.radians(c['adopt_stable_deg']) for w in self.window)
            if strict and stable and span >= c['adopt_window_s'] and mA >= c['adopt_score'] and mA >= mH + c['adopt_gain']:
                event = f'adopt AMCL (mean {mA:.2f} vs held {mH:.2f})'
                if act:
                    self.n['adopt'] += 1
                    self.set_held(self.mo_amcl)
            elif strict and stable and mA >= mH and span < c['adopt_window_s']:
                pass     # AMCL holds steady and matches at least as well: gather evidence before re-seeding it
            elif span >= c['reseed_after_s'] and now - self.last_reseed >= c['reseed_cooldown_s']:
                event = f'reseed AMCL (mean {mA:.2f} vs held {mH:.2f})'
                self.reseed(H, m.header.stamp, ob)
        self.n[state] += 1
        # lost: neither pose matches the map, for window_s
        if c['big_corrections'] == 'strict' and max(sH, sA) < c['lost_score']:
            self.lost_since = self.lost_since if self.lost_since is not None else now
            if now - self.lost_since >= c['window_s'] and now - self.last_search >= c['search_every_s']:
                self.last_search = now
                self.n['search'] += 1
                t0 = time.monotonic()
                best, sb, rival = self.search(H, P)
                ok = sb >= c['search_score'] and sb >= max(sH, sA) + c['search_gain'] and sb >= rival + c['rival_gap']
                mo = compose(best, inverse(ob))
                again = ok and self.candidate is not None and apart(mo, self.candidate)[0] < 0.2 and \
                    apart(mo, self.candidate)[1] < math.radians(3.0)
                self.candidate = mo if ok else None
                event = f'search {sb:.2f} rival {rival:.2f} {"found again" if again else "found" if ok else "no"} ' \
                        f'{time.monotonic() - t0:.2f}s'
                if again:
                    self.n['found'] += 1
                    self.reseed(best, m.header.stamp, ob)
        else:
            self.lost_since = self.candidate = None
        if self.log:
            self.log.write(f'{now:.3f} {sA:.3f} {sH:.3f} {dA:.3f} {math.degrees(dY):.2f} {state} {event}\n')
        if event:
            self.get_logger().info(f'{event}: A {sA:.2f} H {sH:.2f} apart {dA:.2f} m {math.degrees(dY):.1f} deg')
            self.pub_state.publish(String(data=event))

    def reseed(self, pose, stamp, ob):
        """H := pose, and send it to AMCL (its particles restart around it)."""
        if self.c['mode'] == 'passive':
            return
        self.n['reseed'] += 1
        self.set_held(compose(pose, inverse(ob)))
        self.last_reseed = Time.from_msg(stamp).nanoseconds * 1e-9
        m = PoseWithCovarianceStamped()
        m.header.frame_id = self.c['map_frame']
        m.header.stamp = stamp
        self.own_stamp = (stamp.sec, stamp.nanosec)
        m.pose.pose.position.x, m.pose.pose.position.y = pose[0], pose[1]
        m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(pose[2] / 2), math.cos(pose[2] / 2)
        cov = [0.0] * 36
        cov[0] = cov[7] = 0.05 ** 2
        cov[35] = math.radians(3.0) ** 2
        m.pose.covariance = cov
        self.pub_init.publish(m)

    def publish_tf(self):
        if self.mo_held is None:
            return
        t = TransformStamped()
        t.header.stamp = (self.get_clock().now() + Duration(seconds=self.c['tf_future_s'])).to_msg()
        t.header.frame_id, t.child_frame_id = self.c['map_frame'], self.c['odom_frame']
        t.transform.translation.x, t.transform.translation.y = self.mo_held[0], self.mo_held[1]
        t.transform.rotation.z, t.transform.rotation.w = math.sin(self.mo_held[2] / 2), math.cos(self.mo_held[2] / 2)
        self.tfb.sendTransform(t)

    def report(self):
        if self.log:
            self.log.flush()
        if self.n['scans']:
            self.get_logger().info(' '.join(f'{k} {v}' for k, v in self.n.items()))


def main():
    rclpy.init()
    n = Guard()
    try:
        rclpy.spin(n)
    except Exception:   # noqa: BLE001 - shutdown by the launch (SIGINT)
        pass
    finally:
        if n.log:
            n.log.close()
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
