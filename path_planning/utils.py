import json
import os
from typing import List, Tuple
import rclpy
import numpy as np

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseArray, Point
from std_msgs.msg import Header

EPSILON = 0.00000000001

"""
These data structures can be used in the search function.
"""

class LineTrajectory:
    """
    A class to wrap and work with piecewise linear trajectories.
    """

    def __init__(self, node, viz_namespace=None):
        """
        Initialize attirbutes, and create publishers if viz_namespace is not None.
        """
        self.points: List[Tuple[float, float]] = []
        self.distances = []
        self.has_acceleration = False
        self.visualize = False
        self.viz_namespace = viz_namespace
        self.node = node

        # viz_namespace is the namespace where the visualization topics should be located
        if viz_namespace:
            self.visualize = True
            self.start_pub = self.node.create_publisher(
                Marker, viz_namespace + "/start_point", 1
            )
            self.traj_pub = self.node.create_publisher(
                Marker, viz_namespace + "/path", 1
            )
            self.end_pub = self.node.create_publisher(
                Marker, viz_namespace + "/end_pose", 1
            )

    def update_distances(self):
        """
        Compute the distances along the path for all path segments beyond those already computed.
        """
        num_distances = len(self.distances)
        num_points = len(self.points)

        for i in range(num_distances, num_points):
            if i == 0:
                self.distances.append(0)
            else:
                p0 = self.points[i - 1]
                p1 = self.points[i]
                delta = np.array([p0[0] - p1[0], p0[1] - p1[1]])
                self.distances.append(self.distances[i - 1] + np.linalg.norm(delta))

    def distance_to_end(self, t):
        """
        Given a trajectory point index t, compute the distance form this point to the end of the
        path.
        """
        if not len(self.points) == len(self.distances):
            print(
                "WARNING: Different number of distances and points, this should never happen!"
                + " Expect incorrect results. See LineTrajectory class."
            )
        dat = self.distance_along_trajectory(t)
        if dat is None:
            return None

        return self.distances[-1] - dat

    def distance_along_trajectory(self, t):
        """
        Compute distance along path. Ensure path boundaries are respected. The input t is the index
        of a point in the trajectory.
        """
        if t < 0 or t > len(self.points) - 1.0:
            return None
        i = int(t)  # which segment
        t = t % 1.0  # how far along segment
        if t < EPSILON:
            return self.distances[i]

        return (1.0 - t) * self.distances[i] + t * self.distances[i + 1]

    def addPoint(self, point: Tuple[float, float]) -> None:
        """
        Add a point to the trajectory. Points are represented as tuples containing the coordinates
        of a location, of the form (x, y). Coordinates should be expressed in the map frame.
        """
        print("adding point to trajectory:", point)
        self.points.append(point)
        self.update_distances()
        self.mark_dirty()

    def clear(self):
        """
        Remove all points and distances from the trajectory.
        """
        self.points = []
        self.distances = []
        self.mark_dirty()

    def empty(self):
        """
        Return True if the trajectory has no points.
        """
        return len(self.points) == 0

    def save(self, path):
        """
        Save a trajectory to a json file at the given file path. The path must be an absolute path.
        """
        print("Saving trajectory to:", path)
        data = {}
        data["points"] = []
        for p in self.points:
            data["points"].append({"x": p[0], "y": p[1]})
        with open(path, "w") as outfile:
            json.dump(data, outfile)

    def mark_dirty(self):
        """
        Set this attribute to False.
        """
        self.has_acceleration = False

    def dirty(self):
        """
        No acceleration makes a trajectory "dirty", apparently.
        """
        return not self.has_acceleration

    def load(self, path):
        """
        Load a trajectory from a json file at the given absolute path.
        """
        print("Loading trajectory:", path)

        # resolve all env variables in path
        path = os.path.expandvars(path)

        with open(path) as json_file:
            json_data = json.load(json_file)
            for p in json_data["points"]:
                self.points.append((p["x"], p["y"]))
        self.update_distances()
        print("Loaded:", len(self.points), "points")
        self.mark_dirty()

    def fromPoseArray(self, trajMsg):
        """
        Build a trajectory class instance from a trajectory message.
        """
        for p in trajMsg.poses:
            self.points.append((p.position.x, p.position.y))
        self.update_distances()
        self.mark_dirty()
        print("Loaded new trajectory with:", len(self.points), "points")

    def toPoseArray(self):
        """
        Convert a trajectory into a PoseArray object.
        """
        traj = PoseArray()
        traj.header = self.make_header("/map")
        for i in range(len(self.points)):
            p = self.points[i]
            pose = Pose()
            pose.position.x = p[0]
            pose.position.y = p[1]
            traj.poses.append(pose)
        return traj

    def publish_start_point(self, duration=0.0, scale=0.1):
        """
        Publish the start point of the trajectory to the "/start_point" topic, as a Marker message.
        """
        should_publish = len(self.points) > 0
        self.node.get_logger().info("Before Publishing start point")
        if self.visualize and self.start_pub.get_subscription_count() > 0:
            self.node.get_logger().info("Publishing start point")
            marker = Marker()
            marker.header = self.make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 0
            marker.type = 2  # sphere
            marker.lifetime = rclpy.duration.Duration(seconds=duration).to_msg()
            if should_publish:
                marker.action = 0
                marker.pose.position.x = self.points[0][0]
                marker.pose.position.y = self.points[0][1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # delete marker
                marker.action = 2

            self.start_pub.publish(marker)
        elif self.start_pub.get_subscription_count() == 0:
            self.node.get_logger().info("Not publishing start point, no subscribers")

    def publish_end_point(self, duration=0.0):
        """
        Publish the end point of the trajectory to the "/end_pose" topic, as a Marker message.
        """
        should_publish = len(self.points) > 1
        if self.visualize and self.end_pub.get_subscription_count() > 0:
            marker = Marker()
            marker.header = self.make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 1
            marker.type = 2  # sphere
            marker.lifetime = rclpy.duration.Duration(seconds=duration).to_msg()
            if should_publish:
                marker.action = 0
                marker.pose.position.x = self.points[-1][0]
                marker.pose.position.y = self.points[-1][1]
                marker.pose.orientation.w = 1.0
                marker.scale.x = 1.0
                marker.scale.y = 1.0
                marker.scale.z = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                # delete marker
                marker.action = 2

            self.end_pub.publish(marker)
        elif self.end_pub.get_subscription_count() == 0:
            print("Not publishing end point, no subscribers")

    def publish_trajectory(self, duration=0.0):
        """
        Publish the entire trajectory to "/path", as a Marker message.
        """
        should_publish = len(self.points) > 1
        if self.visualize and self.traj_pub.get_subscription_count() > 0:
            self.node.get_logger().info("Publishing trajectory")
            marker = Marker()
            marker.header = self.make_header("/map")
            marker.ns = self.viz_namespace + "/trajectory"
            marker.id = 2
            marker.type = marker.LINE_STRIP  # line strip
            marker.lifetime = rclpy.duration.Duration(seconds=duration).to_msg()
            if should_publish:
                marker.action = marker.ADD
                marker.scale.x = 0.3
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0
                for p in self.points:
                    pt = Point()
                    pt.x = p[0]
                    pt.y = p[1]
                    pt.z = 0.0
                    marker.points.append(pt)
            else:
                # delete
                marker.action = marker.DELETE
            self.traj_pub.publish(marker)
            print("publishing traj")
        elif self.traj_pub.get_subscription_count() == 0:
            print("Not publishing trajectory, no subscribers")

    def publish_viz(self, duration=0):
        """
        Publish everything about the trajectory to be visualized.
        """
        if not self.visualize:
            print("Cannot visualize path, not initialized with visualization enabled")
            return
        self.publish_start_point(duration=duration)
        self.publish_trajectory(duration=duration)
        self.publish_end_point(duration=duration)

    def make_header(self, frame_id, stamp=None):
        """
        Make a header for a message holding trajectory data.
        """
        if stamp is None:
            stamp = self.node.get_clock().now().to_msg()
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id
        return header
