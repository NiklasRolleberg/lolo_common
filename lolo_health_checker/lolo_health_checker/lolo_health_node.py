# General
import yaml
import math
import os
from dataclasses import dataclass, field

# ROS
import rclpy
from rclpy.node import Node
from rclpy import time
from ament_index_python import get_package_share_directory

# Messages
from std_msgs.msg import Int8, Float32, Empty, Bool ,String
from lolo_msgs.msg import Pressures, Status, Temperatures
from ixblue_ins_msgs.msg import Ins

# SMaRC Topics
from lolo_msgs.msg import Topics as LoloTopics
from smarc_msgs.msg import Topics as SmarcTopics


@dataclass
class StatusReport:
    ready: bool = field(default=False)
    fault: bool = field(default=False)


class HealthNode(Node):
    """
    This node will check health related topics and report overall
    status to the BT at a specified rate.
    """
    # TODO - Add the ability to monitor the update rate of sensors

    def __init__(self, namespace=None):
        super().__init__("health_node", namespace=namespace)
        self._log("Starting node defined in lolo_health_node.py")

        # # Desired values
        # # === Load desired values from YAML file
        self.declare_parameter("limits_filename", "lolo_health_limits.yaml")
        self.limits_filename = self.get_parameter("limits_filename").value
        self.limits = self.read_limits()

        # Valid values
        self.valid_pressure_values = None
        self.valid_status_values = None
        self.valid_temperature_values = None
        self.valid_emergency_values = None

        self.set_valid_values()

        # ===== Declare parameters =====
        # Default values set in declare_parameters()
        self.declare_node_parameters()

        # ===== Get parameters =====
        # Note: All parameters must be declared first! see self.declare_parameters
        # === Topics ===
        self.pressure_topic = self.get_parameter("pressure_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.temperature_topic = self.get_parameter("temperature_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.altitude_topic = self.get_parameter("altitude_topic").value

        # TODO - Consider monitoring these topics
        # self.leak_topic = self.get_parameter("leak_topic").value  # Not used
        # self.battery_1_topic = self.get_parameter("battery_1_topic").value  # Not used
        # self.battery_2_topic = self.get_parameter("battery_2_topic").value  # Not used

        # Output topics
        self.output_status_topic = self.get_parameter("output_status_topic").value
        self.output_abort_topic = self.get_parameter("output_abort_topic").value

        # ===== Data ====
        self.current_pressure = None
        self.current_pressure_time = None  # time of last received message
        self.current_status = None
        self.current_status_time = None  # time of last received message
        self.current_temperature = None
        self.current_temperature_time = None  # time of last received message
        self.current_depth = None
        self.current_depth_time = None  # time of last received message
        self.current_altitude = None
        self.current_altitude_time = None
        self.dive_start_time = None
        self.diving = False

        # ===== Status =====
        self.status = SmarcTopics.VEHICLE_HEALTH_WAITING
        self.topics_status = False  # Indicates whether all the monitored topics have been received

        # ===== Behavior Parameters =====
        self.output_rate = self.get_parameter("output_rate").value
        self.debugging = self.get_parameter("debugging").value

        # === Subscriptions ===
        # pressure_topic = LoloTopics.EXTENDED_INTERNAL_PRESSURE_TOPIC
        self.pressure_sub = self.create_subscription(msg_type=Pressures,
                                                     topic=self.pressure_topic,
                                                     callback=self.pressure_callback,
                                                     qos_profile=10)

        # status_topic = LoloTopics.EXTENDED_STATUS_TOPIC
        self.status_sub = self.create_subscription(msg_type=Status,
                                                   topic=self.status_topic,
                                                   callback=self.status_callback,
                                                   qos_profile=10)

        # temperature_topic = LoloTopics.EXTENDED_INTERNAL_TEMPERATURE_TOPIC
        self.temperature_sub = self.create_subscription(msg_type=Temperatures,
                                                        topic=self.temperature_topic,
                                                        callback=self.temperature_callback,
                                                        qos_profile=10)

        self.depth_sub = self.create_subscription(msg_type=Float32,
                                                  topic=self.depth_topic,
                                                  callback=self.depth_callback,
                                                  qos_profile=10)

        self.altitude_sub = self.create_subscription(msg_type=Float32,
                                                  topic=self.altitude_topic,
                                                  callback=self.altitude_callback,
                                                  qos_profile=10)
        # === Publishers ===
        self.check_pub = self.create_publisher(Int8,
                                               SmarcTopics.VEHICLE_HEALTH_TOPIC,
                                               10)

        self.abort_pub = self.create_publisher(msg_type=Empty,
                                               topic=self.output_abort_topic,
                                               qos_profile=10)

        # ===== Timers =====
        self.publisher_timer = self.create_timer(timer_period_sec=float(1.0 / self.output_rate),
                                                 callback=self.publisher_callback)

    def read_limits(self):
        """
        Read YAML file with Lolo's limits.
        Returns a dictionary with the values.
        """
        if not self.limits_filename:
            self.limits_filename = "lolo_health_limits.yaml"

        limits = None
        path_to_pkg = get_package_share_directory('lolo_health_checker')

        yaml_path = os.path.join(path_to_pkg, "config", self.limits_filename)
        with open(path_to_pkg + "/config/" + self.limits_filename, 'r') as file:
            limits = yaml.safe_load(file)
        self.get_logger().info(f"Virtual Lolo has been configured with filename {self.limits_filename}")
        [self.get_logger().info(f"{key}:{value}") for key, value in limits.items()]

        return limits

    def _log(self, message):
        self.get_logger().info(message)

    # Basic node set up
    def declare_node_parameters(self):
        # Declare all the default values for parameters
        # Example: self.declare_parameter("automatic_zone", False)
        # Topics to subscribe to...
        self.declare_parameter("pressure_topic", LoloTopics.EXTENDED_INTERNAL_PRESSURE_TOPIC)
        self.declare_parameter("status_topic", LoloTopics.EXTENDED_STATUS_TOPIC)
        self.declare_parameter("temperature_topic", LoloTopics.EXTENDED_INTERNAL_TEMPERATURE_TOPIC)
        self.declare_parameter("depth_topic", SmarcTopics.DEPTH_TOPIC)
        self.declare_parameter("altitude_topic", SmarcTopics.ALTITUDE_TOPIC)

        # Topics to publish to
        # self.output_status_topic
        self.declare_parameter("output_status_topic", SmarcTopics.VEHICLE_HEALTH_TOPIC)
        self.declare_parameter("output_abort_topic", SmarcTopics.ABORT_TOPIC)

        # Parameters
        self.declare_parameter("output_rate", 1.0)  # Rate (Hz) at with status and abort will be published
        self.declare_parameter("debugging", True)

    def set_valid_values(self):
        """
        Set valid values
        """

        self.valid_pressure_values = {
            "usbl_isb": [250, 1500],
            "thrusters_isb": [250, 1500],
            "vertical_thrusters_isb": [250, 1500],
            "prevco_isb": [250, 1500],
            # "####_isb": [0, 1],
            "battery1_isb": [250, 1500],
            "battery2_isb": [250, 1500],
        }

        self.valid_status_values = {
            # "rc_signal"
            "voltage": [22, 40],
            # "current"
            "captain_leak": [0],
            "esc_leak": [0],
            "prevco_leak": [0],
            "edw_leak": [0],
            "battery1_leak": [0],
            "battery2_leak": [0],

            # ISB status
            "time_status": [1],
            "trigger_status": [1],
            "actuators_status": [1],
            "thrusters_status": [1],
            "vertical_thrusters_status": [1],
            "usbl_status": [1],
            "scientist_status": [1],
            # "edw_status": [1],
            "battery1_status": [1],
            "battery2_status": [1],
        }

        # Temperatures are maybe a little cruder
        temperature_min = -271
        temperature_max = 80
        self.valid_temperature_values = {
            "captain_cpu": [],
            "time_cpu": [],
            "captain_top": [],
            "captain_eth": [],
            "captain_nuc": [],
            "usbl_isb": [],
            "actuator_cpu": [],
            "elevator_pcb": [],
            "elevator_motor": [],
            "rudder_motor": [],
            "rudder_pcb": [],
            "elevon_port_motor": [],
            "elevon_port_pcb": [],
            "elevon_stbd_motor": [],
            "elevon_stbd_pcb": [],
            "thruster_isb": [],
            "port_esc": [],
            'stbd_esc': [],
            "vertical_thruster_isb": [],
            "vertical_thruster_1_esc": [],
            "vertical_thruster_2_esc": [],
            "vertical_thruster_3_esc": [],
            "vertical_thruster_4_esc": [],
            "prevco_isb": [],
            "edw_isb": [],
            "battery1_isb": [],
            "battery1_temp1": [],
            "battery1_temp2": [],
            "battery1_temp3": [],
            "battery1_temp4": [],
            "battery1_temp5": [],
            "battery2_isb": [],
            "battery2_temp1": [],
            "battery2_temp2": [],
            "battery2_temp3": [],
            "battery2_temp4": [],
            "battery2_temp5": [],
        }

        # Simple setting of valid temperature values
        for key in self.valid_temperature_values.keys():
            self.valid_temperature_values[key] = [temperature_min, temperature_max]

        # TODO - Clean ME!!!!!
        self.valid_emergency_values = {
            # NO EMERGENCY is the only "valid" emergency value, all others are scary.
            "emergency_status": "NO EMERGENCY"
        }

    def pressure_callback(self, msg):
        if self.debugging and self.current_pressure is None:
            self._log("pressure_callback()")

        self.current_pressure = msg
        self.current_pressure_time = self.get_clock().now().nanoseconds / 1e9

    def status_callback(self, msg):
        if self.debugging and self.current_status is None:
            self._log("status_callback()")

        self.current_status = msg
        self.current_status_time = self.get_clock().now().nanoseconds / 1e9

    def temperature_callback(self, msg):
        if self.debugging and self.current_temperature is None:
            self._log("temperature_callback()")

        self.current_temperature = msg
        self.current_temperature_time = self.get_clock().now().nanoseconds / 1e9

        # Hack:
        # Sometimes the Servos can send temp=205deg to indicate that something is wrong with the temperature sensor.
        # When it has happended so far it has gone back to normal in the next sample, so we don't want
        # to abort because of that
        if(self.current_temperature.rudder_motor == 205): self.current_temperature.rudder_motor = 20.0
        if(self.current_temperature.rudder_pcb == 205): self.current_temperature.rudder_pcb = 20.0
        if(self.current_temperature.elevator_motor == 205): self.current_temperature.elevator_motor = 20.0
        if(self.current_temperature.elevator_pcb == 205): self.current_temperature.elevator_pcb = 20.0
        if(self.current_temperature.elevon_port_motor == 205): self.current_temperature.elevon_port_motor = 20.0
        if(self.current_temperature.elevon_port_pcb == 205): self.current_temperature.elevon_port_pcb = 20.0
        if(self.current_temperature.elevon_stbd_motor == 205): self.current_temperature.elevon_stbd_motor = 20.0
        if(self.current_temperature.elevon_stbd_pcb == 205): self.current_temperature.elevon_stbd_pcb = 20.0

    def altitude_callback(self, msg):
        self.current_altitude = msg.data
        self.current_altitude_time = self.get_clock().now().nanoseconds / 1e9

    def depth_callback(self, msg):
        self.current_depth = msg.data
        self.current_depth_time = self.get_clock().now().nanoseconds / 1e9

        if "diving_threshold_depth" in self.limits.keys():
            threshold_depth = self.limits["diving_threshold_depth"]
            if self.current_depth > threshold_depth:
                self.diving = True
            else:
                self.diving = False
                self.dive_start_time = self.current_depth_time


    def checker(self, current_msg, current_msg_valid):
        """
        return StatusReport
        """

        status = StatusReport(ready=False, fault=False)

        if current_msg is None:
            # TODO: should this return fault=True?
            return status
        else:
            status.ready = True

        for key, values in current_msg_valid.items():
            # if self.debugging:
            #     self._log(f"Key: {key} -- Values: {values}")
            msg_value = getattr(current_msg, key)
            if len(values) == 1:
                if msg_value != values[0]:
                    status.fault = True
                    if self.debugging:
                        self._log(f"Failure: {key} - {msg_value}")
                    break
            else:
                if not (values[0] <= msg_value <= values[1]):
                    status.fault = True
                    if self.debugging:
                        self._log(f"Failure: {key} - {msg_value}")
                    break

        return status

    def check_pressure(self):
        """
        return StatusReport
        """
        return self.checker(self.current_pressure, self.valid_pressure_values)

    def check_status(self):
        """
        return StatusReport
        """
        return self.checker(self.current_status, self.valid_status_values)

    def check_temperature(self):
        """
        return StatusReport
        """
        return self.checker(self.current_temperature, self.valid_temperature_values)

    def check_emergency(self):
        """
        Check if captain's emergency has been triggered.
        """
        status = StatusReport(ready=False, fault=False)
        if self.current_status is None:
            return status
        if self.current_status.emergency_status == "NO EMERGENCY":
            status.ready = True
            return status
        else:
            status.fault = True
        return status

    def check_depth(self):
        """
        Check if depth exceeds the maximum limit.

        return StatusReport.
        """
        status = StatusReport(ready=False, fault=False)
        if self.current_depth is None:
            return status
        status.ready = True

        if self.current_depth > self.limits["max_depth"]:
            status.fault = True
            self.get_logger().warning(f"WTF are you doing at {self.current_depth} m deep?!")

        return status

    def check_altitude(self):
        """
        Check if altitude exceeds the minimum limit.

        return StatusReport.
        """
        status = StatusReport(ready=False, fault=False)
        if self.current_altitude is None:
            return status

        status.ready = True

        if self.current_altitude < self.limits["min_altitude"]:
            status.fault = True
            self.get_logger().warning(f"Altitude of {self.current_altitude} is too scary, Aborting!")
        return status

    def check_dive_timeout(self):
        """
        Check if Lolo's been diving longer than the max limit.

        return StatusReport.
        """
        status = StatusReport(ready=False, fault=False)
        if self.current_depth is None:
            return status
        else:
            status.ready = True

        if self.diving:
            dive_time = self.current_depth_time - self.dive_start_time
            time_until_timeout = self.limits["max_dive_time"] - dive_time
            self.get_logger().info(f"Diving! Timeout in: {time_until_timeout}")
            if dive_time > self.limits["max_dive_time"]:
                status.fault=True
                self.get_logger().warning(f"Total divetime of {dive_time} reached! Aborting!")

        return status

    def publisher_callback(self):
        """
        Do all the checking here
        """

        pressure_check = self.check_pressure()
        status_check = self.check_status()
        temperature_check = self.check_temperature()
        depth_check = self.check_depth()
        altitude_check = self.check_altitude()
        dive_check = self.check_dive_timeout()

        emergency_check = self.check_emergency()


        ready_checks = [pressure_check.ready, status_check.ready,
                        temperature_check.ready, #emergency_check.ready,
                        depth_check.ready, altitude_check.ready]


        fault_checks = [pressure_check.fault, status_check.fault,
                        temperature_check.fault, #emergency_check.fault,
                        depth_check.fault, altitude_check.fault,
                        dive_check.fault]

        msg = Int8()
        if True in fault_checks:
            msg.data = SmarcTopics.VEHICLE_HEALTH_ERROR
        elif all(ready_checks):
            msg.data = SmarcTopics.VEHICLE_HEALTH_READY
        else:
            msg.data = SmarcTopics.VEHICLE_HEALTH_WAITING
        self.check_pub.publish(msg)

        if self.debugging:
            self._log("publisher_callback()")
            self._log(f"Pressure: {pressure_check}")
            self._log(f"Status: {status_check}")
            self._log(f"Temperature: {temperature_check}")
            self._log(f"Depth: {depth_check}")
            self._log(f"Altitude: {altitude_check}")
            self._log(f"Dive: {dive_check}")
            self._log(f"Emergency: {emergency_check}")
            self._log(f"Output: {msg.data}")

def main(args=None, namespace=None):
    rclpy.init(args=args)
    node = HealthNode(namespace=namespace)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
        rclpy.shutdown()


if __name__ == "__main__":
    default_namespace = "lolo"
    main(namespace=default_namespace)
