import rclpy
from rclpy.node import Node
import os

assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from .utils import LineTrajectory


# TODO:
#   Choose an algorithm to plan trajectories with (probably A*)
#   Write/comment code for selecting one of several algo's (in case others are written)
#   Figure out how the OccupancyGrid message can be used to determine what points are traversable
#   Write the code for the selected algorithm, and make sure that it works
#   Clean up the code w/ pylint and black

class PathPlan(Node):
    """
    Listens for goal pose published by RViz and uses it to plan a path from current car pose.
    """

    def __init__(self):
        """
        Initialize pubs and subs, and create an instance of the LineTrajectory class.
        """
        super().__init__("trajectory_planner")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")
        self.declare_parameter('initial_pose_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )

        self.traj_sub_start = self.create_subscription(
            Marker,
            "/planned_trajectory/start_point",
            self.test_path,
            10
        )

        self.traj_sub_start = self.create_subscription(
            Marker,
            "/planned_trajectory/end_pose",
            self.test_path,
            10
        )

        self.traj_sub = self.create_subscription(
            Marker,
            "/planned_trajectory/path",
            self.test_path,
            10
        )

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

    def map_cb(self, msg):
        """
        Upon receiving a map message, display the data in the simulation, then...
        """
        self.get_logger().info("Recieved map data!")
        self.plan_path()

    def pose_cb(self, pose):
        """
        ...
        """
        self.get_logger().info("Received initlal pose data!")

    def goal_cb(self, msg):
        """
        ...
        """
        self.get_logger().info("Received goal pose data!")

    def plan_path(self, start_point=None, end_point=None, map=None):
        """
        ...
        """
        directory = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.trajectory.load(directory + "/example_trajectories/trajectory_1.traj")
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()

    def test_path(self, msg):
        self.get_logger().info(f"Recieved trajectory data! {msg}")


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
