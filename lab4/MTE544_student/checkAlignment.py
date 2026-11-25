"""
Optional helper (not part of the graded lab code): checks whether the robot's odom
frame and your saved map frame line up.

It takes one laser scan + the robot's odom pose, finds where on the saved map the
scan fits best (like a tiny localizer), and compares that with where odom says the
robot is. If odom and the map share the same (0, 0), the two poses are identical.

Run (Gazebo + the robot publishing /scan and /odom must be running):
    python3 checkAlignment.py                 # uses room.yaml / room.pgm in this folder
    python3 checkAlignment.py --map other     # or another map name

Best test: right after a FRESH Gazebo launch, without driving the robot.
Then odom is (0, 0, 0) and the "map pose" printed below should also be ~(0, 0, 0).
"""
import argparse
import math
import re
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.ndimage import distance_transform_edt

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def load_map(name):
    yaml_text = open(name + ".yaml").read()
    res = float(re.search(r"resolution:\s*([\d.]+)", yaml_text).group(1))
    ox, oy = map(float, re.search(r"origin:\s*\[\s*(-?[\d.]+),\s*(-?[\d.]+)", yaml_text).groups())

    raw = open(name + ".pgm", "rb").read().split(b"\n", 3)
    w, h = map(int, raw[1].split())
    img = np.frombuffer(raw[3], np.uint8).reshape(h, w)
    return img, res, ox, oy


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", default="room", help="map name without extension")
    args = parser.parse_args()

    img, res, mox, moy = load_map(args.map)
    h, w = img.shape
    dist = distance_transform_edt(img >= 10) * res      # metres to nearest wall (pixel <10 = wall)

    def wall_dist(x, y):
        c = np.floor((x - mox) / res).astype(int)
        r = (h - 1 - np.floor((y - moy) / res)).astype(int)
        ok = (c >= 0) & (c < w) & (r >= 0) & (r < h)
        out = np.full(x.shape, 5.0)
        out[ok] = dist[r[ok], c[ok]]
        return out

    # grab one scan and one odom message
    rclpy.init()
    node = Node("check_alignment")
    got = {}
    node.create_subscription(LaserScan, "/scan", lambda m: got.setdefault("scan", m), qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom", lambda m: got.setdefault("odom", m), qos_profile_sensor_data)
    t0 = time.time()
    while ("scan" not in got or "odom" not in got) and time.time() - t0 < 15:
        rclpy.spin_once(node, timeout_sec=0.5)
    if "scan" not in got or "odom" not in got:
        print("no /scan or /odom received - is the simulator running?")
        return
    scan, od = got["scan"], got["odom"]

    q = od.pose.pose.orientation
    oyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    ox_, oy_ = od.pose.pose.position.x, od.pose.pose.position.y

    ranges = np.array(scan.ranges)
    angles = scan.angle_min + np.arange(len(ranges)) * scan.angle_increment
    ok = np.isfinite(ranges) & (ranges > scan.range_min) & (ranges < min(scan.range_max, 3.4))
    ranges, angles = ranges[ok], angles[ok]
    pick = np.linspace(0, len(ranges) - 1, min(70, len(ranges))).astype(int)
    bx, by = ranges[pick] * np.cos(angles[pick]), ranges[pick] * np.sin(angles[pick])

    def score(X, Y, T):
        c, s = np.cos(T)[:, None], np.sin(T)[:, None]
        px, py = X[:, None] + c * bx - s * by, Y[:, None] + s * bx + c * by
        return np.exp(-(wall_dist(px, py) ** 2) / (2 * 0.12 ** 2)).mean(1)

    # coarse search over the whole map, then refine
    xs, ys = np.arange(mox, mox + w * res, 0.15), np.arange(moy, moy + h * res, 0.15)
    XX, YY = [a.ravel() for a in np.meshgrid(xs, ys)]
    free = wall_dist(XX, YY) > 0.2
    XX, YY = XX[free], YY[free]
    best = (0, (0, 0, 0))
    cands = []                                           # strong coarse candidates, to detect ambiguity
    for th in np.radians(np.arange(0, 360, 10)):
        sc = score(XX, YY, np.full(XX.shape, th))
        for i in np.argsort(sc)[-3:]:
            cands.append((sc[i], XX[i], YY[i], th))
        i = sc.argmax()
        if sc[i] > best[0]:
            best = (sc[i], (XX[i], YY[i], th))
    sc0, (x0, y0, t0_) = best
    rivals = [c for c in cands if math.hypot(c[1] - x0, c[2] - y0) > 0.6 and c[0] > sc0 - 0.06]
    for step, span in ((0.05, 0.3), (0.02, 0.08)):
        gx = np.arange(x0 - span, x0 + span + 1e-9, step)
        gy = np.arange(y0 - span, y0 + span + 1e-9, step)
        gt = np.radians(np.arange(-8, 8.1, 1)) + t0_
        G = np.array([(a, b, c) for a in gx for b in gy for c in gt])
        sc = score(G[:, 0], G[:, 1], G[:, 2])
        i = sc.argmax()
        sc0, (x0, y0, t0_) = sc[i], tuple(G[i])

    # transform that maps odom coordinates onto map coordinates
    dyaw = t0_ - oyaw
    tx = x0 - (math.cos(dyaw) * ox_ - math.sin(dyaw) * oy_)
    ty = y0 - (math.sin(dyaw) * ox_ + math.cos(dyaw) * oy_)
    dist_off = math.hypot(tx, ty)

    print(f"map: {args.map}  (resolution {res} m, origin corner {mox}, {moy})")
    print(f"robot pose according to odom:     x={ox_:6.2f}  y={oy_:6.2f}  heading={math.degrees(oyaw):5.0f} deg")
    print(f"robot pose according to the map:  x={x0:6.2f}  y={y0:6.2f}  heading={math.degrees(t0_):5.0f} deg   (scan match {sc0:.2f}, 1.0 = perfect)")
    if rivals:
        r = max(rivals)
        print(f"WARNING: AMBIGUOUS. This scan also fits at x={r[1]:.2f} y={r[2]:.2f} heading={math.degrees(r[3]):.0f} deg "
              f"(score {r[0]:.2f}), almost as well. The numbers below may be wrong.")
        print("         Run it again from a different spot (e.g. in a corner / near distinctive objects).")
    print()
    print(f"offset of odom's (0,0) on the map: x={tx:+.2f} m   y={ty:+.2f} m   rotation={math.degrees(wrap(dyaw)):+.0f} deg")
    if dist_off < 0.15 and abs(math.degrees(wrap(dyaw))) < 5:
        print("=> ALIGNED: odom and the map share the same (0,0). Nothing to fix.")
    else:
        print(f"=> NOT ALIGNED: the map's (0,0) is {dist_off:.2f} m away from the robot's (0,0).")
        if sc0 < 0.6:
            print("   (scan match is weak, so treat these numbers with caution)")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
