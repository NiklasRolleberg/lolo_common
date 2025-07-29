#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import time
from std_msgs.msg import Empty
from geographic_msgs.msg import GeoPoint
from smarc_mission_msgs.srv import GeoFenceChecker
from smarc_mission_msgs.msg import Topics as MissionTopics
from smarc_msgs.msg import Topics as SmarcTopics

class GeoFenceCheckerClient(Node):
    def __init__(self):
        super().__init__('geofence_checker_client')

        self.declare_parameter("verbose", False)
        self.verbose = self.get_parameter("verbose").value

        srv_callback_group = ReentrantCallbackGroup()
        other_callback_group = ReentrantCallbackGroup()
        timer_callback_group = MutuallyExclusiveCallbackGroup()

        self.cli = self.create_client(srv_type=GeoFenceChecker,
                                      srv_name=MissionTopics.GEOFENCE_CHECKER_SERVICE, callback_group=srv_callback_group)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service geofence_checker...')

        self.future = None
        self.req = GeoFenceChecker.Request()

        self.robot_position = None
        self.subscription = self.create_subscription(
            GeoPoint,
            SmarcTopics.POS_LATLON_TOPIC,
            self.latlon_callback,
            1,
            callback_group=timer_callback_group)
        
        self.publisher = self.create_publisher(Empty, SmarcTopics.ABORT_TOPIC, 1, callback_group=other_callback_group)
        self.robot_position = GeoPoint()

        self.timer = self.create_timer(1, self.timer_callback, callback_group=timer_callback_group)


    def latlon_callback(self, msg):
        #self.get_logger().info('\tlatlon callback')
        self.robot_position = msg
        self.req.geopoint = self.robot_position
        if(self.future is None):
            self.future = self.cli.call_async(self.req)

    def timer_callback(self):
        if(self.future == None):
            return
        
        if(self.future.done()):
            result = self.future.result()
            self.get_logger().info('Result: ' + str(result.valid))
            if(result.valid):
                msg = Empty()
                self.publisher.publish(msg)
            self.future = None
        

  
def main():
    rclpy.init()
    client = GeoFenceCheckerClient()
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    executor.spin()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()