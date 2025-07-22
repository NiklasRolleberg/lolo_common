#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

import rclpy
from rclpy.node import Node

import time
from std_msgs.msg import Bool
from geographic_msgs.msg import GeoPoint
from smarc_mission_msgs.srv import GeoFenceChecker
from smarc_mission_msgs.msg import Topics as MissionTopics
from smarc_msgs.msg import Topics as SmarcTopics

class GeoFenceCheckerClient(Node):
    def __init__(self):
        super().__init__('geofence_checker_client')

        self.declare_parameter("verbose", False)
        self.verbose = self.get_parameter("verbose").value

        self.cli = self.create_client(srv_type=GeoFenceChecker,
                                      srv_name=MissionTopics.GEOFENCE_CHECKER_SERVICE)

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service geofence_checker...')

        self.future = None
        self.req = GeoFenceChecker.Request()

        self.robot_position = None
        self.subscription = self.create_subscription(
            GeoPoint,
            SmarcTopics.POS_LATLON_TOPIC,
            self.latlon_callback,
            1)
        
        self.publisher = self.create_publisher(Bool, 'inside_geofence', 10)
        self.robot_position = GeoPoint()


    def latlon_callback(self, msg):
        self.robot_position = msg
        self.req.geopoint = self.robot_position
        if(self.future is None):
            #do service call
            self.future = self.cli.call_async(self.req)


    def spin(self):
        while rclpy.ok():
            time.sleep(0.01)
            rclpy.spin_once(self)
            if(self.future == None):
                continue
            
            if(self.future.done()):
                result = self.future.result()
                self.get_logger().info('Result: ' + str(result.valid))
                msg = Bool()
                msg.data = result.valid
                self.publisher.publish(msg)
                self.future = None

  
def main():
    rclpy.init()
    client = GeoFenceCheckerClient()
    client.spin()
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()