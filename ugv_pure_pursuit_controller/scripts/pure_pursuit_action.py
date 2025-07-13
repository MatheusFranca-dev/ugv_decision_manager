#!/usr/bin/env python3

# ================================================================
# File name: pure_pursuit_action.py
# Description: pure pursuit controller for GEM vehicle in Gazebo,
#              using a ROS Action interface
# Author: Hang Cui (modified by Matheus França)
# Date created: 07/10/2021
# Date modified: 07/12/2025
# Version: 0.1
# Usage: rosrun ugv_pure_pursuit_controller pure_pursuit_action.py
# Python version: 3.8
# ================================================================

import math
import numpy as np
from numpy import linalg as la

import rospy
import actionlib

from std_msgs.msg import Float32
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion

from ugv_bt_interfaces.msg import WaypointAction, WaypointResult


class PurePursuit(object):
    """
    PurePursuit implements a ROS-based pure pursuit controller for Ackermann-steered vehicles.

    This class sets up ROS publishers for crosstrack error and Ackermann drive commands,
    and an action server to receive waypoint goals. It tracks a path using the pure pursuit
    algorithm, computing steering commands to follow a sequence of waypoints.
    The controller stops the robot when the goal is reached or if the action is preempted.

    Attributes:
        rate (rospy.Rate): Control loop rate.
        look_ahead (float): Lookahead distance for pure pursuit (meters).
        wheelbase (float): Vehicle wheelbase (meters).
        goal_dist_thresh (float): Distance threshold to consider goal reached (meters).
        crosstrack_err_pub (rospy.Publisher): Publishes crosstrack error.
        ackermann_pub (rospy.Publisher): Publishes Ackermann drive commands.
        _as (actionlib.SimpleActionServer): Action server for waypoint goals.
    """
    def __init__(self):
        # Variables
        self.rate = rospy.Rate(20)
        self.look_ahead = 6
        self.wheelbase = 1.75
        self.goal_dist_thresh = 1.0

        # Publishers
        self.crosstrack_err_pub = rospy.Publisher('/crosstrack_err',
                                                  Float32, queue_size=1)
        self.ackermann_pub = rospy.Publisher('/gem/ackermann_cmd',
                                             AckermannDrive, queue_size=1)

        # Action server to receive waypoints batch
        self._as = actionlib.SimpleActionServer(
            'waypoint_action', WaypointAction, execute_cb=self.goal_cb,
            auto_start=False)
        self._as.start()

        rospy.loginfo("Waypoints Action Server started, waiting for goal...")

    def stop_robot(self):
        """
        Stops the robot by publishing a zero-speed AckermannDrive command.

        This method creates an AckermannDrive message with both speed and
        steering angle set to zero, publishes it to the robot's command topic,
        and logs the action. This effectively halts the robot's movement.
        """
        stop_msg = AckermannDrive()
        stop_msg.speed = 0.0
        stop_msg.steering_angle = 0.0
        self.ackermann_pub.publish(stop_msg)
        rospy.loginfo("Robot stop command sent.")

    def dist(self, p1, p2):
        """
        Calculate the Euclidean distance between two points in 2D space.

        Args:
            p1 (tuple or list): The first point as (x, y).
            p2 (tuple or list): The second point as (x, y).

        Returns:
            float: The distance between p1 and p2, rounded to three decimal places.
        """
        return round(np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2), 3)

    def find_angle(self, v1, v2):
        """
        Calculates the angle in radians between two vectors.

        Args:
            v1 (array-like): The first vector.
            v2 (array-like): The second vector.

        Returns:
            float: The angle in radians between v1 and v2.
        """
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))

        return np.arctan2(sinang, cosang)

    def get_gem_pose(self):
        """
        Retrieves the current pose (x, y, yaw) of the 'gem' model from Gazebo.

        Waits for the '/gazebo/get_model_state' service to become available,
        then requests the pose of the 'gem' model. If the service call fails,
        returns default values (0.0, 0.0, 0.0).

        Returns:
            tuple: A tuple containing the x position (float), y position (float),
                   and yaw orientation (float), each rounded to 4 decimal places.
        """
        rospy.wait_for_service('/gazebo/get_model_state')

        try:
            service_response = rospy.ServiceProxy(
                '/gazebo/get_model_state', GetModelState)
            model_state = service_response(model_name='gem')
        except rospy.ServiceException as exc:
            rospy.loginfo("Service did not process request: " + str(exc))
            return 0.0, 0.0, 0.0  # Return a default value on failure

        x = model_state.pose.position.x
        y = model_state.pose.position.y

        orientation_q = model_state.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)

        return round(x, 4), round(y, 4), round(yaw, 4)

    def goal_cb(self, goal):
        """
        Callback function executed when a new goal is received by the action server.

        This method implements the main loop for path tracking using the Pure Pursuit algorithm.
        It parses waypoints from the received goal, checks for cancellation requests, monitors
        the vehicle's current pose, and determines if the final goal has been reached. If the goal
        is reached, it stops the robot and signals success to the Behavior Tree. Otherwise, it
        continuously computes the appropriate steering angle and publishes control commands to
        follow the path.

        Args:
            goal: The action goal containing a list of waypoints (with x, y, yaw attributes).

        Behavior:
            - Aborts if no waypoints are provided.
            - Handles preemption requests from the client.
            - Publishes crosstrack error and steering angle for monitoring.
            - Publishes AckermannDrive messages for vehicle control.
            - Signals success to the Behavior Tree upon reaching the final waypoint.
        """
        # Parse waypoints from action goal
        path_points_x = [w.x for w in goal.waypoints]
        path_points_y = [w.y for w in goal.waypoints]
        path_points_yaw = [w.yaw for w in goal.waypoints]

        if not path_points_x:
            rospy.logwarn("Received a goal with no waypoints. Aborting.")
            self._as.set_aborted()
            return

        rospy.loginfo(f"Received {len(path_points_x)} waypoints. Starting pure pursuit.")

        # Main control loop
        while not rospy.is_shutdown():
            # Check if the Behavior Tree has requested a cancellation
            if self._as.is_preempt_requested():
                rospy.loginfo('Action has been preempted by the client.')
                self.stop_robot()
                self._as.set_preempted()
                return

            # Get current vehicle state
            curr_x, curr_y, curr_yaw = self.get_gem_pose()

            # Check if the final goal has been reached
            final_goal_x = path_points_x[-1]
            final_goal_y = path_points_y[-1]
            dist_to_final_goal = self.dist((final_goal_x, final_goal_y), (curr_x, curr_y))

            if dist_to_final_goal < self.goal_dist_thresh:
                rospy.loginfo("Final waypoint reached! Goal has succeeded.")
                self.stop_robot()

                result = WaypointResult()
                result.success = True
                self._as.set_succeeded(result)
                return

            # Pure Pursuit Algorithm
            dist_arr = np.array([self.dist((px, py), (curr_x, curr_y)) for px, py in zip(path_points_x, path_points_y)])

            # Find candidate indices at lookahead distance
            goal_idx = 0  # Default to the first point
            goal_arr = np.where((dist_arr < self.look_ahead + 0.3) & (dist_arr > self.look_ahead - 0.3))[0]
            for idx in goal_arr:
                v1 = [path_points_x[idx]-curr_x, path_points_y[idx]-curr_y]
                v2 = [math.cos(curr_yaw), math.sin(curr_yaw)]
                temp_angle = self.find_angle(v1, v2)
                if abs(temp_angle) < math.pi/2:
                    goal_idx = idx
                    break

            # If no good point is found, it might mean we are far away or passed the path
            # A simple strategy is to target the closest point
            if not goal_arr.any():
                goal_idx = np.argmin(dist_arr)

            L = dist_arr[goal_idx]

            # Transform the goal point to the vehicle's coordinate frame
            gvcx = path_points_x[goal_idx] - curr_x
            gvcy = path_points_y[goal_idx] - curr_y
            goal_x_veh = gvcx * math.cos(curr_yaw) + gvcy * math.sin(curr_yaw)
            goal_y_veh = gvcy * math.cos(curr_yaw) - gvcx * math.sin(curr_yaw)

            # Find the curvature and the angle 
            alpha = path_points_yaw[goal_idx] - curr_yaw
            k = 0.285
            angle_i = math.atan((2 * k * self.wheelbase * math.sin(alpha)) / L)
            angle = angle_i*2
            angle = round(np.clip(angle_i, -0.61, 0.61), 3)

            ct_error = round(math.sin(alpha) * L, 3)
            self.crosstrack_err_pub.publish(ct_error)
            rospy.loginfo_throttle(1.0, f"Crosstrack Error: {ct_error:.3f}, Steering Angle: {angle:.3f}")

            # Publish control command
            ackermann_msg = AckermannDrive()
            ackermann_msg.speed = 2.8
            ackermann_msg.steering_angle = angle
            self.ackermann_pub.publish(ackermann_msg)

            try:
                self.rate.sleep()
            except rospy.ROSInterruptException:
                pass


def pure_pursuit():

    rospy.init_node('pure_pursuit_server_node', anonymous=True)
    pp = PurePursuit()

    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    pure_pursuit()
