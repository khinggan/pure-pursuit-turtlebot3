#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Point
from visualization_msgs.msg import Marker
import numpy as np
import math
import tf

## Turtlebot3 Parameters
MAX_LINEAR_SPEED = 0.22
MAX_ANGULAR_SPEED = 2.84

class PurePursuitTurtlebot3:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('pure_pursuit_turtlebot3', anonymous=True)

        self.robot_pose = Pose()

        # Pure Pursuit Algorithm Parameters
        self.Kp_rho = 3
        self.Kp_alpha = 8
        self.Kp_beta = -1.5
        self.WaypointType = "ellipse"    # Waypoints type: ellipse or sin
        self.waypoints = self.get_waypoints(self.WaypointType)
        self.lookahead_distance = 0.3   # [m] Pure Pursuit Algorithm Parameters
        self.read_threshold = 1.0    # [m] # Reach threshold, sum formation error is lower then this, move to next target virtual structure
        self.old_nearest_point_index = None
        self.last_index = len(self.waypoints[0]) - 1
        self.target_idx = 0    # target index, target virtual structure can be got from this. 
        self.target_angle = 0.0
        
        # Create odom subscriber
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Create a velocity publisher
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Path Visualization publisher
        self.waypoints_publisher = rospy.Publisher("waypoints", Marker, queue_size=10)
        
        # Set a rate for publishing messages
        self.rate = rospy.Rate(10) # 10 Hz

    def odom_callback(self, data):
        self.robot_pose = data.pose.pose           # extract pose from odom data

    def pure_pursuit(self):
        twist = Twist()
        while not rospy.is_shutdown():
            # Get target point on waypoints
            tx, ty = self.get_target_point()
            
            rospy.loginfo("(tx: %f, ty: %f), (cx: %f, cy: %f)", tx, ty, self.robot_pose.position.x, self.robot_pose.position.y)
            # Drive to target point (move2pose)
            quaternion = (self.robot_pose.orientation.x, self.robot_pose.orientation.y, self.robot_pose.orientation.z, self.robot_pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)
            target_theta = math.atan2(ty-self.robot_pose.position.y, tx-self.robot_pose.position.x)
            v, w = self.move2pose(tx, ty, target_theta, self.robot_pose.position.x, self.robot_pose.position.y, euler[2])

            if abs(v) > MAX_LINEAR_SPEED:
                v= np.sign(v) * MAX_LINEAR_SPEED
            if abs(w) > MAX_ANGULAR_SPEED:
                w = np.sign(w) * MAX_ANGULAR_SPEED

            # Publish v, w
            twist.linear.x = v
            twist.angular.z = w
            self.velocity_publisher.publish(twist)

            # visualize
            self.waypoint_rviz()
            self.rate.sleep()
    
    def get_target_point(self):
        # Parameters: current position, look ahead distance
        # Return: target_point x, y, z
        cx = self.robot_pose.position.x
        cy = self.robot_pose.position.y

        if self.old_nearest_point_index is None:
            dx = [cx - icx for icx in self.waypoints[0]]
            dy = [cy - icy for icy in self.waypoints[1]]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = math.hypot(self.waypoints[0][ind] - cx,
                                            self.waypoints[1][ind] - cy)
            while True:
                if (ind + 1) >= len(self.waypoints[0]):
                    break
                distance_next_index = math.hypot(self.waypoints[0][ind + 1] - cx,
                                                self.waypoints[1][ind + 1]- cy)
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.waypoints[0]) else ind
                distance_this_index = distance_next_index
                
            self.old_nearest_point_index = ind
        # Lf = k * self.twist.linear.x + Lfc  # use Changing look ahead distance
        Lf = self.lookahead_distance          # or use Fixed size look ahead distance

        # search look ahead target point index
        while Lf > math.hypot(self.waypoints[0][ind] - cx, self.waypoints[1][ind] - cy):
            if (ind + 1) >= len(self.waypoints[0]):
                break  # not exceed goal
            ind += 1

        self.target_idx = ind

        # get target angle
        dy = self.waypoints[1][self.target_idx] - cx
        dx = self.waypoints[0][self.target_idx] - cy
        self.target_angle = math.atan2(dy, dx)

        return self.waypoints[0][self.target_idx], self.waypoints[1][self.target_idx]

    def get_waypoints(self, waypoint_type):
        '''
        waypoint_type: `sin` or `ellipse` path waypoints
        ellipse: 2a = 20, 2b = 10
        sin: long = 20, width = 13
        return: path X (list), path Y (list)

        self.lookahead_distance should > nearest two waypoint distance.
        the density of the waypoints are affect the path tracking performance

        sin: path_X = np.arange(0, 20, 0.1); 0.1 affect density
        ellipse: thetas = np.linspace(-np.pi, np.pi, 200); 200 affect density

        '''
        path_X, path_Y = None, None
        if waypoint_type == "sin":
            path_X = np.arange(0, 20, 0.1)
            path_Y = [math.sin(ix / 2.0) * ix / 2.0 for ix in path_X]
        elif waypoint_type == "ellipse":
            thetas = np.linspace(-np.pi, np.pi, 200)
            a, b = 5, 2.5
            path_X = [a * np.cos(theta) + a for theta in thetas]
            path_Y = [b * np.sin(theta) for theta in thetas]
        else:
            path_X = [0.0]
            path_Y = [0.0]
            print("{} TYPE of waypoints not exist".format(waypoint_type))
        return path_X, path_Y

    def move2pose(self, x_goal, y_goal, theta_goal, x, y, theta):
        """
        Constructs an instantiate of the PathFinderController for navigating a
        3-DOF wheeled robot on a 2D plane
        Parameters
        ----------
        Kp_rho : The linear velocity gain to translate the robot along a line
                towards the goal
        Kp_alpha : The angular velocity gain to rotate the robot towards the goal
        Kp_beta : The offset angular velocity gain accounting for smooth merging to
                the goal angle (i.e., it helps the robot heading to be parallel
                to the target angle.)
        Returns the control command for the linear and angular velocities as
            well as the distance to goal
            Parameters
            ----------
            x_diff : The position of target with respect to current robot position
                    in x direction
            y_diff : The position of target with respect to current robot position
                    in y direction
            theta : The current heading angle of robot with respect to x axis
            theta_goal: The target angle of robot with respect to x axis
            Returns
            -------
            rho : The distance between the robot and the goal position
            v : Command linear velocity
            w : Command angular velocity
            

            Description of local variables:
            - alpha is the angle to the goal relative to the heading of the robot
            - beta is the angle between the robot's position and the goal
            position plus the goal angle
            - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
            the goal
            - Kp_beta*beta rotates the line so that it is parallel to the goal
            angle
            
            Note:
            we restrict alpha and beta (angle differences) to the range
            [-pi, pi] to prevent unstable behavior e.g. difference going
            from 0 rad to 2*pi rad with slight turn
        """
        x_diff = x_goal - x
        y_diff = y_goal - y
        
        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        v = 0.0 if rho < 0.2 else self.Kp_rho * rho
        w = 0.0 if v == 0 else self.Kp_alpha * alpha + self.Kp_beta * beta

        if np.pi / 2 < alpha < np.pi or -np.pi < alpha < -np.pi / 2:
            v = -v     
        return v, w
    
    def waypoint_rviz(self):
        # visualize waypoints
        waypoints = []
        for x, y in zip(self.waypoints[0], self.waypoints[1]):
            waypoint = Point()
            waypoint.x = x
            waypoint.y = y
            waypoint.z = 0.0
            waypoints.append(waypoint)

        waypoints_visualize = Marker()
        waypoints_visualize.header.frame_id = "odom"
        waypoints_visualize.header.stamp = rospy.Time.now()
        waypoints_visualize.ns = "path"
        waypoints_visualize.id = 0
        waypoints_visualize.action = Marker.ADD
        waypoints_visualize.pose.orientation.w=0.0
        waypoints_visualize.color.r = 0.0
        waypoints_visualize.color.g = 0.0
        waypoints_visualize.color.b = 1.0
        waypoints_visualize.color.a = 1.0
        waypoints_visualize.scale.x = 0.1
        waypoints_visualize.type = 4

        waypoints_visualize.points = waypoints

        self.waypoints_publisher.publish(waypoints_visualize)

if __name__ == '__main__':
    try:
        pure_pursuit_turtlebot3 = PurePursuitTurtlebot3()
        pure_pursuit_turtlebot3.pure_pursuit()
    except rospy.ROSInterruptException:
        pass
