"""
Optional helper (not part of the graded lab code): shows the robot in RViz.

Subscribes to /odom and publishes
  * /robotPoseMarker : a circle the size of the robot, with a short arrow from its
                       centre showing the heading
  * /actualPath      : the trajectory the robot has actually driven (nav_msgs/Path),
                       the counterpart of the green planned path on /designedPath

The actual path is cleared every time a new goal arrives on /goal_pose, so each goal
gets its own driven trajectory.

Run it in its own terminal:   python3 robotPoseMarker.py
In RViz (already in pathPlanner.rviz): Marker display on /robotPoseMarker and
Path display on /actualPath.

Note: this shows the raw odometry pose. The map->odom transform published by
mapPublisher.py is the identity, so the odom pose is the pose on the map.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker

ROBOT_DIAMETER = 0.22      # [m] circle drawn for the robot (TurtleBot3 burger is ~0.18 m wide)
ARROW_LENGTH = 0.35        # [m] heading arrow, starts at the robot's centre
MIN_STEP = 0.03            # [m] only add a point to the trajectory after moving this far
MAX_POINTS = 5000          # keep the path message from growing forever
PATH_RATE = 5.0            # [Hz] how often the trajectory is published


class RobotPoseMarker(Node):

    def __init__(self):
        super().__init__("robot_pose_marker")

        self.markerPublisher = self.create_publisher(Marker, "/robotPoseMarker", 10)
        self.pathPublisher = self.create_publisher(Path, "/actualPath", 10)

        self.create_subscription(Odometry, "/odom", self.odomCallback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/goal_pose", self.newGoalCallback, 10)

        self.path = Path()
        self.path.header.frame_id = "map"      # odom == map (identity transform from mapPublisher.py)
        self.lastXY = None

        self.create_timer(1.0 / PATH_RATE, self.publishPath)

    def newGoalCallback(self, msg: PoseStamped):
        # start a fresh trajectory for the new goal
        self.path.poses.clear()
        self.lastXY = None

    def odomCallback(self, msg: Odometry):

        pose = msg.pose.pose
        stamp = msg.header.stamp

        # --- circle with an arrow ---
        circle = Marker()
        circle.header.frame_id = "map"
        circle.header.stamp = stamp
        circle.ns = "robot_pose"
        circle.id = 0
        circle.type = Marker.CYLINDER           # a flat disc seen from above
        circle.action = Marker.ADD
        circle.pose = pose
        circle.scale.x = ROBOT_DIAMETER
        circle.scale.y = ROBOT_DIAMETER
        circle.scale.z = 0.03
        circle.color.r, circle.color.g, circle.color.b, circle.color.a = 1.0, 0.1, 0.1, 0.9

        arrow = Marker()
        arrow.header.frame_id = "map"
        arrow.header.stamp = stamp
        arrow.ns = "robot_pose"
        arrow.id = 1
        arrow.type = Marker.ARROW               # from the robot's centre, along its heading
        arrow.action = Marker.ADD
        arrow.pose = pose
        arrow.pose.position.z = pose.position.z + 0.03
        arrow.scale.x = ARROW_LENGTH
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 1.0, 1.0

        self.markerPublisher.publish(circle)
        self.markerPublisher.publish(arrow)

        # --- driven trajectory ---
        x, y = pose.position.x, pose.position.y
        if self.lastXY is None or math.hypot(x - self.lastXY[0], y - self.lastXY[1]) >= MIN_STEP:
            point = PoseStamped()
            point.header.frame_id = "map"
            point.header.stamp = stamp
            point.pose = pose
            self.path.poses.append(point)
            if len(self.path.poses) > MAX_POINTS:
                self.path.poses.pop(0)
            self.lastXY = (x, y)

    def publishPath(self):
        if not self.path.poses:
            return
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.pathPublisher.publish(self.path)


def main():
    rclpy.init()
    node = RobotPoseMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
