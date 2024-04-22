import rclpy
from rclpy.node import Node
import os
import math

assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from .utils import LineTrajectory


# TODO:
# [X]    Choose an algorithm to plan trajectories with (probably A*)
# [X]    Write/comment code for selecting one of several algo's (in case others are written)
# [X]    Figure out how the OccupancyGrid message can be used to determine what points are traversable
# [X?]    Write the code for the selected algorithm, and make sure that it works
# []    If other algorithms are written, find a way to evaluate each and determine the best one
#           Consider the relative length of returned paths to the objective distance between the start
#           and goal, as well as how long it takes for each function to run in seconds (The asymptotic
#            runtime should be well known, and we want the fastest algorithm specifically for the purposes of this lab).
#           Eventually we could use z-scores to determine the best algorithm, but that's only if we have time.
# []    Clean up the code w/ pylint and black

class PathPlan(Node):
    """
    Listens for goal pose published by RViz and uses it to plan a path from current car pose.
    """

    def __init__(self):
        """
        Initialize pubs, subs, and class attributes, and create an instance of the LineTrajectory class.
        """
        super().__init__("trajectory_planner")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")
        self.declare_parameter('initial_pose_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value

        # Dictionary of map information
        # width (int) and height (int) are the dimensions of the map in grid cells
        # resolution (float) is the length in meters that each grid cell represents in the map
        # origin (Pose) is the position and orientation of cell (0, 0) in the map
        # grid (list[int]) is a list of integers in row-major order, where each cell's integer represents the probability of there being an obstacle in that cell
        self.map = {
            "width": None,
            "height": None,
            "resolution": None,
            "origin": None,
            "grid": None
        }

        # These control how the received map is broken up into a graph for the search-based algorithms to traverse.
        self.grid_type = 2
        self.step_length = 5
        self.directions = [(self.step_length*math.cos(i*math.pi/self.grid_type), self.step_length*math.sin(i*math.pi/self.grid_type))
                           for i in range(0, self.grid_type*2)]
        
        # These control what algorithm is used to calculate the shortest path between points
        self.path_finders = [self.example_path, self.bfs, self.a_star]
        self.pf_select = 0

        # These are the start and goal point of the path, represented as tuples of the form (x, y)
        self.start_point = None
        self.goal_point = None


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

        self.traj_sub_end = self.create_subscription(
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
        Upon receiving a map message, pass the data into the self.map dictionary, then attempt to run a path-planning algorithm.
        """
        self.get_logger().info("Recieved map data!")
        self.map["width"] = msg.info.width
        self.map["height"] = msg.info.height
        self.map["resolution"] = msg.info.resolution
        self.map["origin"] = msg.info.origin
        self.map["grid"] = msg.data

        # x = self.map["origin"].position.x
        # y = self.map["origin"].position.y
        # self.get_logger().info(f"{self.get_pixel_at_real_coords(x, y)=}")

        if self.start_point is not None and self.goal_point is not None:
            self.path_finders[self.pf_select]()
        # self.example_path()

    def pose_cb(self, msg):
        """
        Upon receiving the initial pose, pass the position coordinates into self.start_point, then attempt to run a path-planning algorithm.
        """
        self.get_logger().info("Received initial pose data!")
        position = msg.pose.pose.position
        self.start_point = (position.x, position.y)
        if self.map["grid"] is not None and self.goal_point is not None:
            self.path_finders[self.pf_select]()

    def goal_cb(self, msg):
        """
        Upon receiving the goal pose, pass the position coordinates into self.goal_point, then attempt to run a path-planning algorithm.
        """
        self.get_logger().info("Received goal pose data!")
        position = msg.pose.pose.position
        self.goal_point = (position.x, position.y)
        if self.map["grid"] is not None and self.start_point is not None:
            self.path_finders[self.pf_select]()

    def test_path(self, msg):
        self.get_logger().info(f"Received trajectory data!")

    # --- Map Traversal Functions ---
    def get_pixel_at_real_coords(self, x, y):
        """
        Given a pair of real-world coordinates, retrieve the index of the corresponding pixel in self.map["grid"].
        The value at this index is the likelihood that the real-world coordinate contains an obstacle.
        Assume that the origin's orientation is a quaternion of the form [0, 0, 1, angle (rad)]
        """
        map_pose = self.map["origin"]
        angle = map_pose.orientation.w
        px = map_pose.position.x
        py = map_pose.position.y

        # POTENTIAL BUG: Possible incorrect translation from real-world coordinates to pixel coordinates
        u = (x-px)*math.cos(-angle) - (y-py)*math.sin(-angle)
        v = (x-px)*math.sin(-angle) + (y-py)*math.cos(-angle)

        return int(round(v*self.map["resolution"])*self.map["width"] + (u*self.map["resolution"]))

    def distance(self, point_a, point_b):
        """
        Return the distance between two points, represented as tuples in the form (x, y).
        """
        return math.sqrt((point_b[0]-point_a[0])**2 + (point_b[1]-point_a[1])**2)
    

    def can_move(self, point_a, point_b):
        """
        Find all points in self.map["grid"] that contain a part of the line segment between point_a and point_b.
        Then calculate the probability that no points have an obstacle, and return True if this probability
        is over 0.5.
        """
        # POTENTIAL BUG: This method might not work properly

        reso = self.map["resolution"]
        coords = {}
        rx = point_b[0]-point_a[0]
        ry = point_b[1]-point_a[1]
        m = ry/rx
        b = point_b[1] - m*point_b[0]
        # x_dir and y_dir are each either 1 or -1
        x_dir = int(rx/abs(rx))
        y_dir = int(ry/abs(ry))


        # Find the x-values where the line crosses between different cells in the grid, and save the two cells in a set
        x_values = [i*reso for i in range(math.ceil(point_a[0]/reso), math.ceil(point_b[0]/reso), x_dir)]
        for x_val in x_values:
            coords.add((x_val-reso/2, m*(x_val-reso/2)+b))
            coords.add((x_val+reso/2, m*(x_val+reso/2)+b))

        # Find the y-values where the line crosses between different cells in the grid, and save the two cells in a set
        y_values = [j*reso for j in range(math.ceil(point_a[1]/reso), math.ceil(point_b[1]/reso), y_dir)]
        for y_val in y_values:
            coords.add((((y_val-reso/2)-b)/m, y_val-reso/2))
            coords.add((((y_val+reso/2)-b)/m, y_val+reso/2))

        # Find the indices of all collected coordinates, the calculate the desired probability
        indices = {self.get_pixel_at_real_coords(*coord) for coord in coords}
        prob_empty = 1
        for index in indices:
            prob_empty *= (self.map["grid"][index] - 1)
        return (prob_empty > 0.5)


    # --- Path Planner Function(s) ---
    # The function(s) below will attempt to calculate the shortest feasible path between a start
    # point and an end point. Each function should take in as input the messages for the initial
    # pose, the goal pose, and the obstacle data, and publish a PoseArray message containing the
    # shortest path between the two points (or None if no path is possible). A function may have
    # other inputs depending on what it does, and a few new parameters will likely need to be created.

    # TODO A* (w/ square, triangular, or hexagonal grid, and variable distance between consecutive nodes)
    # Iterate through a queue of nodes until the goal is found. Make sure to check in between nodes for the goal too.
    def a_star(self):
        """
        Use the A* algorithm to create the shortest path between self.start_point and self.end_point (both represented as tuples of real_world coordinates).
        Publish the path as a trajectory, or raise an error if there is no possible shortest path.
        """

    # TODO Sampling-Based Planner (I don't know what this could be yet, we can figure it out later)

    # TODO Another Search-Based Planner (It's possible that BFS or Djikstra is the better algorithm over A* specifically for driving
    #   a racecar around. Either algo might run much faster, with the small cost of a slightly longer path. We should consider them
    #   both even when A* seems like the best choice.)

    def bfs(self):
        """
        Use the BFS algorithm to create the shortest path between self.start_point and self.end_point (both represented as tuples of real_world coordinates).
        Publish the path as a trajectory, or raise an error if there is no possible shortest path.
        """
        reached_from = {self.start_point: None}
        to_check = [self.start_point]
        checked = {}

        while to_check:
            current = to_check.pop(0)
            checked.add(current)
            # Check if current is close enough to end_point to move directly to it
            if self.distance(current, self.goal_point) < self.step_length:
                if self.can_move(current, self.goal_point):
                    # Build the shortest path
                    path = [self.goal_point, current]
                    next_point = reached_from[current]
                    while next_point is not None:
                        path.append(next_point)
                        next_point = reached_from[next_point]
                    path.reverse()
                    self.trajectory.clear()
                    for point in path:
                        self.trajectory.addPoint(point)
                    self.traj_pub.publish(self.trajectory.toPoseArray())
                    self.trajectory.publish_viz()
                    return None
            # Otherwise, find the neighbors of the current point and queue them to be checked next
            neighbors = [(current[0] + direction[0], current[1] + direction[1]) for direction in self.directions]
            for neighbor in neighbors:
                if self.can_move(current, neighbor) and neighbor not in checked:
                    to_check.append(neighbor)

        raise Exception("No valid path between points")
                    

    def example_path(self):
        """
        Publish an example path to be displayed.
        """
        directory = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
        self.trajectory.load(directory + "/example_trajectories/trajectory_1.traj")
        # self.trajectory.addPoint(start_point)
        # self.trajectory.addPoint(end_point)
        # self.trajectory.addPoint((10.0, 25.0))
        # self.trajectory.addPoint((25.0, 10.0))
        # self.trajectory.addPoint((4.0, 4.0))
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
