import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from tf2_msgs.msg import TFMessage #https://docs.ros.org/en/melodic/api/tf2_msgs/html/msg/TFMessage.html

from .utils import LineTrajectory

import numpy as np


class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic = self.get_parameter('drive_topic').get_parameter_value().string_value

        self.lookahead = 1  # FILL IN #
        self.speed = 1.0  # FILL IN #
        self.wheelbase_length = 0  # FILL IN #

        self.trajectory = LineTrajectory("/followed_trajectory")
        self.initialized_traj = False

        self.traj_sub = self.create_subscription(PoseArray,
                                                 "/trajectory/current",
                                                 self.trajectory_callback,
                                                 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped,
                                               self.drive_topic,
                                               1)
        
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic,
                                                 self.odom_pose_callback,
                                                 1)
        
        self.target_pub = self.create_publisher(PoseStamped,
                                                "/targetpose",
                                                1)
        self.current_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                 "currentpose",
                                                 1)
        
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # create a listener to pose broadcasting
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                                 self.clicked_pose_callback,
                                                 1)
        
        self.update_target_freq = 30.0 # in hertz
        self.create_timer(1/self.update_target_freq, self.update_target)
        
        self.get_logger().info("finished init, waiting for trajectory...")

    # updates member variables for pose when new odometry data is received
    def odom_pose_callback(self, odom):
        self.x = odom.pose.pose.position.x
        self.y = odom.pose.pose.position.y
        self.yaw = quaternion_to_euler(odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, odom.pose.pose.orientation.y, odom.pose.pose.orientation.z)[2]

    # updates member variables for pose when new clicked data is received
    def clicked_pose_callback(self, update_pose):
        # https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseWithCovarianceStamped.html
        
        # broadcast clicked pose for visualization
        self.visualize_pose(update_pose)
        
        # updating member variables
        self.x = update_pose.pose.pose.position.x
        self.y = update_pose.pose.pose.position.y

        orientation = update_pose.pose.pose.orientation
        orientation = quaternion_to_euler(orientation.w, orientation.x, orientation.y, orientation.z)

        self.yaw = orientation[2] # we just want the yaw

        self.get_logger().info("Clicked pose to x: %f y: %f theta: %f" % (self.x, self.y, self.yaw))

    def update_target(self):
        if (not self.initialized_traj): # don't update target if trajectory not initialized lol
            return
        
        self.point = self.find_next_point()
        return

    # Finds the next point on the trajectory that we should be traveling to
    # If point does not exist (within lookahead distance), returns None
    # Otherwise, returns point as list [x, y, yaw]
    def find_next_point(self):
        points = self.trajectory.points
        num_points = len(points)

        min_dist = self.min_dist_squared(points[0], points[1], [self.x, self.y])
        min_dist_i = 0
        for i in range(num_points-1):
            # compute distance to segment between points i and i+1
            dist = self.min_dist_squared(points[i], points[i+1], [self.x, self.y])
            if (dist < min_dist):
                min_dist = dist 
                min_dist_i = i
        
        self.get_logger().info(f"Closest segment found: ({points[min_dist_i][0]}, {points[min_dist_i][1]}) to ({points[min_dist_i+1][0]}, {points[min_dist_i+1][1]})")

        # min_dist_i is the index of the closest segment to the robot 
        point = [False, None, None]
        max_ahead = 5   # can only look this many segments ahead from min_dist_i
        ahead = 0

        center_circle = [self.x, self.y]
        
        # while we haven't found a valid point,
        # we haven't looked ahead more than max_ahead segments,
        # and we haven't reached the final point in the trajectory:
        while(point[0] == False and ahead < max_ahead and ahead + min_dist_i < num_points-1):
            segment = [
                [points[min_dist_i+ahead][0], points[min_dist_i+ahead][1]],
                [points[min_dist_i+ahead+1][0], points[min_dist_i+ahead+1][1]]
            ]
            self.get_logger().info(f"Looking {ahead} steps ahead, on segment {segment}")
            point = self.intersection_point(segment, center_circle)
            ahead += 1
        
        if (point[0] == False): # no point was found lol
            self.get_logger().info("No point was found :(")
            return None
    
        # point was found! visualize and return it
        point = [point[1][0], point[1][1], point[2]]
        self.visualize_target_pose(point)
        self.get_logger().info(f"Point found! {point}")
        return 

    # Receives and processes trajectory
    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        self.get_logger().info("printing trajectory points:")
        # self.get_logger().info(len(self.trajectory.points))
        for point in self.trajectory.points:
            self.get_logger().info(f"{point[0]} {point[1]}")

        self.initialized_traj = True
    
    
    # VISUALIZATION HELPER FUNCTIONS -------------------------------------------------

    # visualizes our current pose with topic /currentpose
    # with a circle around it with the correct lookahead radius
    # (abusing covariance field of PoseWithCovarianceStamped lol)
    def visualize_pose(self, pose):
        pose.pose.covariance[0] = 1.0 * self.lookahead
        pose.pose.covariance[7] = 1.0 * self.lookahead
        pose.pose.covariance[35] = 0.0
        self.current_pub.publish(pose)
        return
    
    # visualizes our target pose with topic /targetpose
    def visualize_target_pose(self, point):
        target_pose = PoseStamped()
        target_pose.header.frame_id = "/map"
        target_pose.pose.position.x = point[0]
        target_pose.pose.position.y = point[1]
        quat = euler_to_quaternion(0, 0, point[2])
        target_pose.pose.orientation.x = quat[1]
        target_pose.pose.orientation.y = quat[2]
        target_pose.pose.orientation.z = quat[3]
        target_pose.pose.orientation.w = quat[0]
        self.target_pub.publish(target_pose)
        
    
    # GEOMETRY HELPER FUNCTIONS -------------------------------------------------
    # TODO convert these to numpy operations :)
    
    # start, end, and point are all vectors
    # Returns the minimum squared distance between point and the segment from start to end
    def min_dist_squared(self, start, end, point):
        start = np.array(start)
        end = np.array(end)
        point = np.array(point)
        
        dist_squared = np.dot(end-start, end-start)

        if (dist_squared == 0): return np.dot(point-start, point-start) # TODO REPLACE

        t = max(0, min(1, np.dot(point-start, end-start) / dist_squared))
        proj = start + t * (end - start) # Projection falls on the segment
        return np.dot(proj-point, proj-point)
    
    # segment is a list of two vectors, center_circle is a vector
    # Returns the point of intersection between the circle centered at center_circle with lookahead radius
    # and the line between the two vectors in segments
    # If exists, returns [True, [x,y], yaw]
    # If false, returns [False, None, None]
    def intersection_point(self, segments, center_circle):
        # https://codereview.stackexchange.com/questions/86421/line-segment-to-circle-collision-algorithm/86428#86428 

        p1 = np.array(segments[0])
        p2 = np.array(segments[1])
        v = p2-p1
        q = center_circle
        r = self.lookahead
        
        angle = np.arctan2(v[1], v[0])
        
        a = np.dot(v, v)
        b = 2 * (np.dot(v, p1-q))
        c = np.dot(p1, p1) + np.dot(q, q) - 2 * np.dot(p1, q) - r**2
        
        disc = b**2 - 4 * a * c

        # if not real discriminant (no solution)
        if disc < 0:
            return False, None, None
        
        sqrt_disc = np.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        # t2 = (-b - sqrt_disc) / (2 * a)
        # we don't care about t2 because the corresponding point to it (p1+t2*v)
        # will always be "behind" the robot
        
        # TODO handle edge case where we are "past" the end target point
        # possibly add a "backing up" procedure
        
        if (0 <= t1 <= 1):
            return True, p1+t1*v, angle
        else:
            return False, None, None
    
    # FRAME TRANSFORMATION HELPER FUNCTIONS -------------------------------------------------
    
    def world_to_robot_frame(self, world_pose, robot_pose):
        theta = robot_pose[2]
        next_pose = robot_pose[0:2] + np.array([np.cos(theta) * world_pose[0] + -np.sin(theta) * world_pose[1], 
                                                np.sin(theta) * world_pose[0] + np.cos(theta) * world_pose[1]])
        next_theta = (theta + world_pose[2]) % (2 * 3.14159)

        return np.array([next_pose[0], next_pose[1], next_theta])

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
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()



    