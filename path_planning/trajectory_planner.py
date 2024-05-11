import rclpy
from rclpy.node import Node
import os
import math
from queue import PriorityQueue
import numpy as np
# import scipy.signal as magic
import random


assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from .utils import LineTrajectory


# TODO:
# [X]    Choose an algorithm to plan trajectories with (probably A*)
# [X]    Write/comment code for selecting one of several algo's (in case others are written)
# [X]    Figure out how the OccupancyGrid message can be used to determine what points are traversable
# [X]    Write the code for the selected algorithm, and make sure that it works
# [X?]    If other algorithms are written, find a way to evaluate each and determine the best one
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
        # grid (list[int]) is a list of integers in row-major order. Each cell's integer represents the probability of there being an obstacle in that cell,
        # and each integer is in the range [0, 100] for probabilities from 0 to 1, or is -1 if the probability is unknown
        self.map = {
            "width": None,
            "height": None,
            "resolution": None,
            "origin": None,
            "grid": None
        }

        # These control how the received map is broken up into a graph for the search-based algorithms to traverse.
        self.grid_type = 2
        self.step_length = 1
        # self.directions = [(self.step_length*math.cos(i*math.pi/self.grid_type), self.step_length*math.sin(i*math.pi/self.grid_type))
        #                    for i in range(0, self.grid_type*2)]
        self.directions = []
        for x in [-1.0, 0.0, 1.0]:
            for y in [-1.0, 0.0, 1.0]:
                if(not (x==0 and y == 0)):
                    self.directions.append((self.step_length*x/(2)**0.5, self.step_length*y/(2)**0.5))
        
        # These control what algorithm is used to calculate the shortest path between points
        self.path_finders = [self.example_path, self.bfs, self.a_star, self.RRT]
        self.pf_select = 2

        # These are the start and goal point of the path, represented as tuples of the form (x, y)
        self.start_point = None
        self.goal_point = None

        self.x = 0
        self.y = 0
        self.yaw = 0


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

        self.odom_sub = self.create_subscription(
            Odometry,
            '/pf/pose/odom',
            self.odom_cb,
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

        self.goal_has_been_set= False
        self.counter_started = False
        self.counter = 0

        # self.timer = self.create_timer(0.1, self.start_planning_timer_callback)

    def start_planning_timer_callback(self):
        if(self.goal_has_been_set):
            if(self.counter_started):
                self.counter+=1
            else:
                self.counter_started = True
                self.counter+=1
        if(self.counter == 20):
            self.path_finders[self.pf_select]()
            self.goal_has_been_set = False
            self.counter_started = False
            self.counter = 0
        # self.get_logger().info(f'Timer callback executed {self.counter} times')

    def map_cb(self, msg):
        """
        Upon receiving a map message, pass the data into the self.map dictionary, then attempt to run a path-planning algorithm.
        """
        self.get_logger().info("Received map data!")
        self.map["width"] = msg.info.width
        self.map["height"] = msg.info.height
        self.map["resolution"] = msg.info.resolution
        self.map["origin"] = msg.info.origin
        self.map["grid"] = msg.data

        # # if you want to blur
        # data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
        # dim = 20
        # convolution = np.ones((dim, dim))/(dim**2)
        # blurred_data = magic.convolve2d(data, convolution, mode="same", boundary="fill", fillvalue=100)
        # blurred_in_og_shape = blurred_data.reshape(-1)
        # self.map["grid"] = blurred_in_og_shape


        # self.map["wxh_grid"]=data
        
        # map width: 1730
        # height: 1300
        # resolution: 0.0504
        # origin: 25.9, 48.5
        # orientation: idk some rotation
        
        # x = self.map["origin"].position.x
        # y = self.map["origin"].position.y
        # self.get_logger().info(f"{self.get_pixel_at_real_coords(x, y)=}")

        # if self.start_point is not None and self.goal_point is not None:
        #     self.path_finders[self.pf_select]()
        # self.example_path()

    def pose_cb(self, msg):
        """
        Upon receiving the initial pose, pass the position coordinates into self.start_point, then attempt to run a path-planning algorithm.
        """
        self.get_logger().info("Received initial pose data!")
        position = msg.pose.pose.position
        self.start_point = (position.x, position.y)
        # if self.map["grid"] is not None and self.goal_point is not None:
        #     self.path_finders[self.pf_select]()

    def odom_cb(self, msg):
        # self.get_logger().info("Received odom data!")
        odom = msg
        self.start_point = (odom.pose.pose.position.x, odom.pose.pose.position.y)

    def goal_cb(self, msg):
        """
        Upon receiving the goal pose, pass the position coordinates into self.goal_point, then attempt to run a path-planning algorithm.
        """
        self.get_logger().info("Received goal pose data!")
        position = msg.pose.position
        self.goal_point = (position.x, position.y)
        if self.map["grid"] is not None and self.start_point is not None:
            self.get_logger().info(f"{self.start_point}")
            self.get_logger().info(f"{self.goal_point}")
            self.goal_has_been_set= True
            self.path_finders[self.pf_select]()
            # self.get_logger().info("TIMER CALLBACK TILL PLANNING STARTS INITIATED!")


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
        euler = quaternion_to_euler(map_pose.orientation.w, map_pose.orientation.x, map_pose.orientation.y, map_pose.orientation.z)
        angle = euler[2]
        px = map_pose.position.x
        py = map_pose.position.y
        # self.get_logger().info(f"xy {x} {y}")
        # self.get_logger().info(f"pxpy {px} {py}")
        # self.get_logger().info(f"angle {angle}")

        # POTENTIAL BUG: Possible incorrect translation from real-world coordinates to pixel coordinates
        u = (x)*math.cos(-angle) - (y)*math.sin(-angle) + px
        v = (x)*math.sin(-angle) + (y)*math.cos(-angle) + py
        
        # self.get_logger().info(f"uv {u} {v}")

        str1 =np.array2string(np.round(u/self.map["resolution"]), separator=', ')
        str2 =np.array2string(np.round(v/self.map["resolution"]), separator=', ')
        
        # self.get_logger().info(f"uv {str1} {str2}")
        
        return np.round(v/self.map["resolution"])*self.map["width"] + (np.round(u/self.map["resolution"]))

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

        number_of_points = 5

        x_values = rx*np.arange(0, 1, 1/number_of_points)+point_a[0]
        y_values = ry*np.arange(0, 1, 1/number_of_points)+point_a[1]
        # self.get_logger().info(f"xy: {x_values} {y_values}")
        indices = self.get_pixel_at_real_coords(x_values, y_values)
        
        # m = ry/rx
        # b = point_b[1] - m*point_b[0]
        # # x_dir and y_dir are each either 1 or -1

        # x_dir = int(rx/abs(rx))
        # y_dir = int(ry/abs(ry))
        
        # self.get_logger().info(f"directions: {rx} {ry} {x_dir} {y_dir}")

        # # Find the x-values where the line crosses between different cells in the grid, and save the two cells in a set
        # x_values = [i*reso for i in range(math.ceil(point_a[0]/reso), math.ceil(point_b[0]/reso), x_dir)]
        # for x_val in x_values:
        #     coords.append((x_val-reso/2, m*(x_val-reso/2)+b))
        #     coords.append((x_val+reso/2, m*(x_val+reso/2)+b))

        # # Find the y-values where the line crosses between different cells in the grid, and save the two cells in a set
        # y_values = [j*reso for j in range(math.ceil(point_a[1]/reso), math.ceil(point_b[1]/reso), y_dir)]
        # for y_val in y_values:
        #     coords.append((((y_val-reso/2)-b)/m, y_val-reso/2))
        #     coords.append((((y_val+reso/2)-b)/m, y_val+reso/2))

        # # Find the indices of all collected coordinates, the calculate the desired probability
        # indices = {self.get_pixel_at_real_coords(*coord) for coord in coords}
        
        # self.get_logger().info(f"indices: {indices}")
        
        # prob_empty = 1
        for index in list(indices):
            val = self.map["grid"][int(index)]
            if (val >0 or val == -1): return False
            # self.get_logger().info(f"indices: {val}")
            # prob_empty *= (self.map["grid"][int(index)] - 1)
        # return (prob_empty > 0.5), prob_empty
        return True
    
    def cost_to_move(self, point_a, point_b):
        cost = self.distance(point_a, point_b)
        return cost

    def valid_neighbor(self, coord):
        return coord >= 0 and coord < self.map["width"] * self.map["height"]

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
        frontier = PriorityQueue()
        frontier.put((0, self.start_point))
        reached_from = {self.start_point: None}


        # for pre cost we store int(10^6 times float)
        pre_cost = {}
        def hash(point):
            return (int(point[0]*10e6), int(point[1]*10e6))
        pre_cost[hash(self.start_point)] = 0
        checked = {}
        points_checked_counter = 0
        while not frontier.empty():
            current = frontier.get()[1]

            points_checked_counter+=1
            # self.get_logger().info(f"current: {current}")
            # self.get_logger().info(f"loss: {self.distance(current, self.goal_point)}")
            
            #check if we are within a critical distance of the goal
            #if so,
            if (self.distance(current, self.goal_point) <= 2 * self.step_length and self.can_move(current, self.goal_point)):
                reached_from[self.goal_point] = current
                break

            neighbors = [(current[0] + direction[0], current[1] + direction[1]) for direction in self.directions]
            neighbor_check = []
            for neighbor in neighbors:
                # self.get_logger().info(f"n: {neighbor}")
                # if self.valid_neighbor(neighbor) and self.can_move(current, neighbor):
                if self.can_move(current, neighbor):
                    neighbor_check.append(neighbor)
                # prob=self.can_move(current, neighbor)[1]
                # self.get_logger().info(f"prob: {prob}")

            for next in neighbor_check:
                #CHANGED
                # new_cost = pre_cost[hash(current)] + self.step_length
                new_cost = pre_cost[hash(current)] + self.distance(current, next)
                already_checked = hash(next) in list(pre_cost.keys())
                if not already_checked or already_checked and new_cost < pre_cost[hash(next)]:
                    pre_cost[hash(next)] = new_cost
                    priority = new_cost + self.distance(self.goal_point, next)
                    # self.get_logger().info(f"priorities: {priority} next {next} precost {new_cost}")
                    frontier.put((priority, next))
                    reached_from[next] = current

        self.get_logger().info(f"NUMBER OF TOTAL POINTS CHECKED: {points_checked_counter}")
        self.get_logger().info("path found, reconstructing trajectory")
        self.get_logger().info(f"start and goal: {self.start_point} {self.goal_point}")
        curr = reached_from[self.goal_point]
        traj = [curr]
        self.get_logger().info(f"{curr} {traj}")
        while (not curr == self.start_point):
            curr = reached_from[curr]
            self.get_logger().info(f"curr: {curr}")
            traj.append(curr)
        self.get_logger().info(f"trajectory: {traj}")
        
        pose_array_msg = PoseArray()
        pose_array_msg.header.frame_id = "/map"
        for i in range(len(traj)-1, -1, -1):
            p = Pose()
            p.position.x = traj[i][0]
            p.position.y = traj[i][1]
            pose_array_msg.poses.append(p)

        self.get_logger().info(f" {pose_array_msg}, publishing!!!!!!!!!!!!!!!!!")
        
        self.traj_pub.publish(pose_array_msg)
        
        # self.get_logger().info(f"{pre_cost.keys()}")
        
        # for val in pre_cost.keys():
        #     p = Pose()
        #     p.position.x = val[0]
        #     p.position.y = val[1]
        #     pose_array_msg.poses.append(p)
        # self.traj_pub.publish(pose_array_msg)

    
    # TODO Sampling-Based Planner (I don't know what this could be yet, we can figure it out later)
    def RRT(self):
        """
        Use the RRT algorithm to create the shortest path between self.start_point and self.end_point.
        Publish the path as a trajectory, or raise an error if there is no possible shortest path.
        """
        # POTENTIAL BUG: Random points calculated weirdly...
        def random_coord():
            range = max(self.map["width"], self.map["height"])*self.map["resolution"]
            return (random.random()*2 - 1)*range
        
        reached_from = {self.start_point: None}
        points = [self.start_point]
        while self.goal_point not in reached_from.keys():
            # Choose a point to move to
            if random.random() > 0.9:
                to_point = self.goal_point
            else:
                rand_x = random_coord() + self.map["origin"].position.x
                rand_y = random_coord() + self.map["origin"].position.y
                to_point = (rand_x, rand_y)
            try:
                # Find a point to add to the tree
                points.sort(key=lambda p: self.distance(p, to_point))
                closest_point = points[0]
                to_dist = self.distance(closest_point, to_point)
                if to_dist <= self.step_length and self.can_move(closest_point, to_point):
                    reached_from[to_point] = closest_point
                    points.append(to_point)
                else:
                    scale = self.step_length/to_dist
                    step_point = (c+(t-c)*scale for c, t in zip(closest_point, to_point))
                    if self.can_move(closest_point, step_point):
                        reached_from[step_point] = closest_point
                        points.append(step_point)
            except:
                pass
        # Build path
        path = []
        next = self.goal_point
        while next is not None:
            path.append(next)
            next = reached_from[next]
        path.reverse()
        self.trajectory.clear()
        for point in path:
            self.trajectory.addPoint(point)
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()
        
            


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
        checked = set()

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
        
def quaternion_to_euler(w, x, y, z):

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = np.sqrt(1 + 2 * (w * y - x * z))
    cosp = np.sqrt(1 - 2 * (w * y - x * z))
    pitch = 2 * np.arctan2(sinp, cosp) - np.pi / 2

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return [roll, pitch, yaw]

# helper function to convert from euler angles to quaternion
def euler_to_quaternion(roll, pitch, yaw):

    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w,x,y,z]

def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()
