
from mapUtilities import *
from a_star import *

POINT_PLANNER=0; TRAJECTORY_PLANNER=1

# heuristic used by A*: True = Euclidean, False = Manhattan
USE_EUCLIDEAN=True

class planner:
    def __init__(self, type_, mapName="room_trial1"):

        self.type=type_
        self.mapName=mapName

    
    def plan(self, startPose, endPose):
        
        if self.type==POINT_PLANNER:
            return self.point_planner(endPose)
        
        elif self.type==TRAJECTORY_PLANNER:
            self.costMap=None
            self.initTrajectoryPlanner()
            return self.trajectory_planner(startPose, endPose)


    def point_planner(self, endPose):
        return endPose

    def initTrajectoryPlanner(self):


        # TODO PART 5 Create the cost-map, the laser_sig is 
        # the standard deviation for the gausiian for which
        # the mean is located on the occupant grid. 
        # cost > 0.8 (treated as wall by A*) within ~0.67*laser_sig of an obstacle,
        # so 0.3 gives ~0.2 m clearance (TurtleBot 4 radius ~0.17 m)
        self.m_utilites=mapManipulator(laser_sig=0.4)
            
        self.costMap=self.m_utilites.make_likelihood_field()
        

    def trajectory_planner(self, startPoseCart, endPoseCart):


        # This is to convert the cartesian coordinates into the 
        # the pixel coordinates of the map image, remmember,
        # the cost-map is in pixels. You can by the way, convert the pixels
        # to the cartesian coordinates and work by that index, the a_star finds
        # the path regardless. 
        startPose=self.m_utilites.position_2_cell(startPoseCart)
        endPose=self.m_utilites.position_2_cell(endPoseCart)
        
        # run A* on the cost map (in cells)
        pathCells = search(self.costMap, startPose, endPose, USE_EUCLIDEAN)

        if pathCells is None:
            print("no path found")
            return None

        # TODO PART 5 convert the cell pixels into the cartesian coordinates

        Path = list(map(self.m_utilites.cell_2_position, pathCells))



        # TODO PART 5 return the path as list of [x,y]
        return [[x, y] for x, y in Path]




def generate_test_map(basename, res=0.05, origin=(-1.0, -1.0)):
    """
    Write a synthetic map (basename.pgm + basename.yaml) in the same format
    map_saver_cli produces: a 6 m x 4 m room with an inner wall that has a
    doorway near the top, plus a box obstacle. Free = 254, occupied = 0.
    """
    import os
    os.makedirs(os.path.dirname(basename) or ".", exist_ok=True)

    width, height = int(6.0 / res), int(4.0 / res)        # 120 x 80 pixels
    img = np.full((height, width), 254, dtype=np.uint8)   # rows = image y (top = 0)

    img[:2, :] = 0; img[-2:, :] = 0                       # outer walls
    img[:, :2] = 0; img[:, -2:] = 0

    def col(x): return int(round((x - origin[0]) / res))
    def row(y): return int(round(height - (y - origin[1]) / res))

    # inner wall at x = 2.0 m, with a doorway between y = 1.8 m and y = 2.6 m
    img[:, col(2.0):col(2.0) + 2] = 0
    img[row(2.6):row(1.8), col(2.0):col(2.0) + 2] = 254

    # box obstacle from (0.8, -0.2) to (1.4, 0.6)
    img[row(0.6):row(-0.2), col(0.8):col(1.4)] = 0

    with open(basename + ".pgm", "wb") as f:
        f.write(f"P5\n{width} {height}\n255\n".encode())
        f.write(img.tobytes())

    with open(basename + ".yaml", "w") as f:
        f.write(f"image: {os.path.basename(basename)}.pgm\n"
                f"mode: trinary\n"
                f"resolution: {res:.6f}\n"
                f"origin: [{origin[0]:.6f}, {origin[1]:.6f}, 0.000000]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.25\n")

    print(f"generated test map: {basename}.pgm / .yaml")


if __name__=="__main__":

    # you can use this part of the code to test your
    # search algorithm regardless of the ros2 hassles
    #
    # examples:
    #   python3 planner.py --generate                         # synthetic map, default start/goal
    #   python3 planner.py --map room --start 0 0 --goal 2 1  # your real map (metres)

    import argparse
    import time
    import matplotlib.pyplot as plt

    parser = argparse.ArgumentParser(description="offline test of the A* trajectory planner")
    parser.add_argument("--map", type=str, default="room", help="map name/path (with or without .yaml)")
    parser.add_argument("--generate", action="store_true", help="generate and use a synthetic test map")
    parser.add_argument("--start", type=float, nargs=2, default=[0.0, 0.0], metavar=("X", "Y"), help="start in metres")
    parser.add_argument("--goal", type=float, nargs=2, default=[4.0, 0.0], metavar=("X", "Y"), help="goal in metres")
    parser.add_argument("--out", type=str, default="planner_test.png", help="where to save the figure")
    parser.add_argument("--no-show", action="store_true", help="only save the figure, don't open a window")
    args = parser.parse_args()

    if args.generate:
        args.map = "generated_map/room"
        generate_test_map(args.map)

    # mapManipulator is a ROS node, so rclpy has to be initialized (nothing is spun or published)
    rclpy.init()

    # build the cost map once (same steps as initTrajectoryPlanner, but with a
    # selectable map file), then reuse it for both heuristics
    p = planner(TRAJECTORY_PLANNER)
    p.m_utilites = mapManipulator(args.map, laser_sig=0.3)
    p.costMap = p.m_utilites.make_likelihood_field()
    mu = p.m_utilites

    # warn if start/goal fall in the blocked band (A* would fail / search the whole map)
    for name, pt in (("start", args.start), ("goal", args.goal)):
        c = mu.position_2_cell(pt)
        if not (0 <= c[0] < mu.width and 0 <= c[1] < mu.height):
            print(f"WARNING: {name} {pt} is outside the map")
        elif p.costMap[c[1], c[0]] > 0.8:
            print(f"WARNING: {name} {pt} is too close to an obstacle (cost {p.costMap[c[1], c[0]]:.2f} > 0.8)")

    # plan with both heuristics
    results = {}
    for label, euclid in (("Euclidean", True), ("Manhattan", False)):
        USE_EUCLIDEAN = euclid         # module-level switch read by trajectory_planner
        t0 = time.time()
        path = p.trajectory_planner(args.start, args.goal)
        dt = time.time() - t0

        if path is None:
            print(f"{label}: no path found ({dt:.2f} s)")
            continue

        pts = np.array(path)
        length = np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))
        print(f"{label}: {len(path)} waypoints, length {length:.2f} m, planned in {dt:.2f} s")
        results[label] = pts

    # plot: cost map in metres with both paths overlaid
    o_x, o_y = mu.o_x, mu.o_y
    extent = [o_x, o_x + mu.width * mu.res, o_y, o_y + mu.height * mu.res]

    fig, ax = plt.subplots(figsize=(9, 6))
    ax.imshow(p.costMap, cmap="gray_r", extent=extent, origin="upper", vmin=0, vmax=1)

    styles = {"Euclidean": ("tab:blue", "-"), "Manhattan": ("tab:orange", "--")}
    for label, pts in results.items():
        color, ls = styles[label]
        ax.plot(pts[:, 0], pts[:, 1], ls, color=color, linewidth=2, label=f"A* path ({label})")

    ax.plot(*args.start, "o", color="tab:green", markersize=10, label="Start")
    ax.plot(*args.goal, "*", color="tab:red", markersize=15, label="Goal")

    ax.set_title(f"A* planned paths on {args.map}")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.4)
    ax.legend(loc="best")
    ax.set_aspect("equal")

    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    print(f"saved figure to {args.out}")

    if not args.no_show:
        plt.show()

    rclpy.shutdown()

