import math
import os
import random
import subprocess
import time
from os import path

import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

TIME_DELTA = 0.1


class GazeboEnv(Node):
    """Superclass for all Gazebo environments."""

    def __init__(self, launchfile, environment_dim):
        super().__init__('gazebo_env')
        
        
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.scan_array = []
        self.prev_lin_vel = 0.0


        self.launch_gazebo_simulation(fullpath)
        # subscription of odom, scan 
        self.unpause_client = self.create_client(Empty, "/gazebo/unpause_physics")
        self.pause_client = self.create_client(Empty, "/gazebo/pause_physics")
        self.reset_client = self.create_client(Empty, "/gazebo/reset_world")
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = self.euler_from_quaternion(msg.pose.pose.orientation)
    def scan_callback(self, msg):
        # sub to /scan topic and extract 30 points(90/3), from msg[-45:45:3] maybe
        self.scan_array = msg[-45:45:3]

    def launch_gazebo_simulation(self, fullpath):
        subprocess.Popen(["ros2", "launch", "neo_simulation2", "simulation.launch.py"])
        self.get_logger().info("Gazebo launched!")

    def step(self, action):
        self.call_service(self.unpause_client)
        time.sleep(TIME_DELTA)
        self.call_service(self.pause_client)
        # return state, reward, done, target
        return 0

    def reset(self):
        """Reset the state of the environment and return an initial observation."""
        self.get_logger().info("Resetting the environment...")

        self.call_service(self.reset_client)
        self.call_service(self.unpause_client)
        time.sleep(TIME_DELTA)
        self.call_service(self.pause_client)
        self.get_logger().info("Environment reset complete.")
        return 0

        # return state
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

    def call_service(self, client):
        """Helper method to call ROS2 services"""
        if not client.service_is_ready():
            client.wait_for_service()
        
        req = Empty.Request()
        client.call_async(req)


    @staticmethod
    def get_reward(target, collision, action, min_laser):
        # return rew
        return 0


