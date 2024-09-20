import math
import os
import random
import subprocess
import time
from os import path
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs import LaserScan

TIME_DELTA = 0.1
Collid_check = 0.05
Goal_reached = 0.25

class GazeboEnv(Node):
    """Superclass for all Gazebo environments."""

    def __init__(self, launchfile):
        super().__init__('gazebo_env')
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.theta = 0.0
        self.prev_linvel = 0.0
        self.prev_angvel = 0.0
        self.scan_data = []
        self.goal_x = 2.0
        self.goal_y = 3.0 
        
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback_callback,
            10
        )
        
        # Launch the Gazebo simulation
        self.launch_gazebo_simulation(fullpath)
        # Create clients for ROS2 services
        self.unpause_client = self.create_client(Empty, "/unpause_physics")
        self.pause_client = self.create_client(Empty, "/pause_physics")
        self.reset_client = self.create_client(Empty, "/reset_simulation")
        self.reset_world = self.create_client(Empty, "reset_world")
    def scan_callback(self, msg):
        self.scan_data = min(msg.ranges[-45:45:3]) # expect 30 values
    def odom_callback(self, msg):
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        self.theta = self.euler_from_quaternion(msg.pose.pose.orientation)
    
    def launch_gazebo_simulation(self, fullpath):
        self.gazebo_process = subprocess.Popen(["ros2", "launch", "neo_simulation2", "simulation.launch.py"])
        self.get_logger().info("Gazebo launched!")

    def step(self, action):
        """Perform an action and read a new state"""
        self.get_logger().info(f"Performing step with action: {action}")
        
        self.prev_linvel = action[0]
        self.prev_angvel = action[1]
        self.call_service_sync(self.unpause_client)
        time.sleep(TIME_DELTA)
        self.call_service_sync(self.pause_client)
        done, collision, min_laser = self.observe_collision(self.velodyne_data)
        v_state = []
        v_state[:] = self.velodyne_data[:]
        laser_state = [v_state]

        distance = np.linalg.norm(
            [self.odom_x - self.goal_x, self.odom_y - self.goal_y]
        )
        # Calculate the relative angle between the robots heading and heading toward the goal
        skew_x = self.goal_x - self.odom_x
        skew_y = self.goal_y - self.odom_y
        dot = skew_x * 1 + skew_y * 0
        mag1 = math.sqrt(math.pow(skew_x, 2) + math.pow(skew_y, 2))
        mag2 = math.sqrt(math.pow(1, 2) + math.pow(0, 2))
        beta = math.acos(dot / (mag1 * mag2))
        if skew_y < 0:
            if skew_x < 0:
                beta = -beta
            else:
                beta = 0 - beta
        theta = beta - self.theta
        if theta > np.pi:
            theta = np.pi - theta
            theta = -np.pi - theta
        if theta < -np.pi:
            theta = -np.pi - theta
            theta = np.pi - theta

        # Detect if the goal has been reached and give a large positive reward
        if distance < Goal_reached:
            target = True
            done = True

        robot_state = [distance, theta, action[0], action[1]]
        state = np.append(self.scan_data, robot_state)
        reward = self.get_reward(target, collision, action, min_laser)
        return state, reward, done, target


    def reset(self):
        """Reset the state of the environment and return an initial observation."""
        self.get_logger().info("Resetting the environment...")
        self.call_service_sync(self.reset_client)
        self.call_service_sync(self.unpause_client)
        time.sleep(TIME_DELTA)
        self.call_service_sync(self.pause_client)
        self.get_logger().info("Environment reset complete.")
        return 0  # Placeholder return
    def euler_from_quaternion(self, quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw 
    def call_service_sync(self, client):
        """Helper method to call ROS2 services synchronously."""
        if not client.service_is_ready():
            self.get_logger().info(f"Waiting for service {client.srv_name} to become available...")
            client.wait_for_service()
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")
    @staticmethod
    def observe_collision(laser_data):
        # Detect a collision from laser data
        min_laser = min(laser_data)
        if min_laser < Collid_check:
            return True, True, min_laser
        return False, False, min_laser

    def turn_off(self):
        """Turn off the simulation"""
        self.get_logger().info("Turning off the simulation...")
        self.call_service_sync(self.pause_client)
        self.shutdown()

    @staticmethod
    def get_reward(target, collision, action, min_laser):
        """Compute and return the reward"""
        reward = 0

        return reward

    def shutdown(self):
        """Shutdown the Gazebo process"""
        if hasattr(self, 'gazebo_process'):
            self.gazebo_process.terminate()
            self.gazebo_process.wait()
            self.get_logger().info("Gazebo process terminated.")