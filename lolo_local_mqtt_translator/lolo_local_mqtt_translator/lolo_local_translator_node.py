import rclpy
import json
import time
import subprocess
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Float32
from geographic_msgs.msg import GeoPoint
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import BatteryState
from lolo_msgs.msg import Status
from lolo_msgs.msg import Pressures
from lolo_msgs.msg import Temperatures
from lolo_msgs.msg import VescFeedback
import paho.mqtt.client as mqtt


class Translator_node(Node):
    def __init__(self, mqtt_node):
        super().__init__('translator_node')
        self.mqtt = mqtt_node
        self._last_published = {}

        # Existing subscriptions
        self.subscription_menu_out = self.create_subscription(
            String, '/lolo/debug/menu_out', self.callback_menu_out, 2)
        self.subscription_internal_pressure = self.create_subscription(
            Pressures, '/lolo/extended/internal_pressure', self.callback_internal_pressure, 2)
        self.subscription_internal_temperature = self.create_subscription(
            Temperatures, '/lolo/extended/internal_temperature', self.callback_internal_temperature, 2)
        self.subscription_status = self.create_subscription(
            Status, '/lolo/extended/status', self.callback_status, 2)

        # Navigation subscriptions
        self.subscription_speed = self.create_subscription(
            Float32, '/lolo/smarc/speed', self.callback_speed, 2)
        self.subscription_depth = self.create_subscription(
            Float32, '/lolo/smarc/depth', self.callback_depth, 2)
        self.subscription_heading = self.create_subscription(
            Float32, '/lolo/smarc/heading', self.callback_heading, 2)
        self.subscription_course = self.create_subscription(
            Float32, '/lolo/smarc/course', self.callback_course, 2)
        self.subscription_latlon = self.create_subscription(
            GeoPoint, '/lolo/smarc/latlon', self.callback_latlon, 2)

        # Battery subscriptions
        self.subscription_battery1 = self.create_subscription(
            BatteryState, '/lolo/extended/battery1', self.callback_battery1, 2)
        self.subscription_battery2 = self.create_subscription(
            BatteryState, '/lolo/extended/battery2', self.callback_battery2, 2)

        # GPS fix subscription
        self.subscription_navsatfix = self.create_subscription(
            NavSatFix, '/lolo/standard/navsatfix', self.callback_navsatfix, 2)

        # Thruster RPM feedback (VescFeedback). All thrusters share the same
        # message type, so one generic callback handles them all and the latest
        # values are aggregated into a single MQTT topic /lolo/extended/thrusters.
        # Map: ROS topic -> short key used in the MQTT payload.
        self.thruster_topics = {
            '/lolo/extended/actuators/thruster_port_fb': 'port',
            '/lolo/extended/actuators/thruster_stbd_fb': 'strb',
            # --- vertical thrusters: confirm the EXACT ROS topic names, then uncomment ---
            # '/lolo/extended/actuators/vertical_thruster_1_fb': 'vt1',
            # '/lolo/extended/actuators/vertical_thruster_2_fb': 'vt2',
            # '/lolo/extended/actuators/vertical_thruster_3_fb': 'vt3',
            # '/lolo/extended/actuators/vertical_thruster_4_fb': 'vt4',
        }
        self._thruster_state = {}
        self._thruster_subs = []
        for ros_topic, key in self.thruster_topics.items():
            sub = self.create_subscription(
                VescFeedback, ros_topic,
                lambda msg, k=key: self.callback_thruster(msg, k), 2)
            self._thruster_subs.append(sub)

        # Connectivity timer — runs every 10 seconds
        self.connectivity_timer = self.create_timer(5.0, self.callback_connectivity)

    def _should_publish(self, topic, interval_seconds=1.0):
        """Returns True if enough time has passed since last publish for this topic."""
        now = time.time()
        last = self._last_published.get(topic, 0)
        if now - last >= interval_seconds:
            self._last_published[topic] = now
            return True
        return False

    def _ping(self, host, timeout=1):
        """Returns True if host responds to ping, False otherwise."""
        try:
            result = subprocess.run(
                ['ping', '-c', '1', '-W', str(timeout), host],
                capture_output=True,
                timeout=timeout + 1
            )
            return result.returncode == 0
        except Exception:
            return False

    def callback_connectivity(self):
        """Pings both communication paths and publishes connectivity status."""
        try:
            # Ubiquiti link — ping the Teltonika side of the point-to-point link
            ubiquiti_up = self._ping('192.168.1.6')

            # 4G link — ping Google DNS as internet reachability check
            fourG_up = self._ping('8.8.8.8')

            # Determine active canal
            if ubiquiti_up:
                active_canal = 'UBIQUITI'
            elif fourG_up:
                active_canal = '4G'
            else:
                active_canal = 'NONE'

            payload = json.dumps({
                "ubiquiti_up": ubiquiti_up,
                "fourG_up":    fourG_up,
                "active_canal": active_canal
            })
            self.mqtt.publish("/lolo/connectivity", payload)
            self.get_logger().info(f'Published connectivity: {payload}')
        except Exception as e:
            print(e)

    def callback_menu_out(self, msg):
        if not self._should_publish('/lolo/debug/menu_out', 2.0):
            return
        try:
            payload = json.dumps({
                "menu_out": msg.data
            })
            self.mqtt.publish("/lolo/debug/menu_out", payload)
            self.get_logger().info(f'Published menu_out: {payload}')
        except Exception as e:
            print(e)

    def callback_speed(self, msg):
        if not self._should_publish('/lolo/smarc/speed', 2.0):
            return
        try:
            payload = json.dumps({
                "speed": round(msg.data, 4)
            })
            self.mqtt.publish("/lolo/smarc/speed", payload)
            self.get_logger().info(f'Published speed: {payload}')
        except Exception as e:
            print(e)

    def callback_depth(self, msg):
        if not self._should_publish('/lolo/smarc/depth', 2.0):
            return
        try:
            payload = json.dumps({
                "depth": round(msg.data, 4)
            })
            self.mqtt.publish("/lolo/smarc/depth", payload)
            self.get_logger().info(f'Published depth: {payload}')
        except Exception as e:
            print(e)

    def callback_heading(self, msg):
        if not self._should_publish('/lolo/smarc/heading', 2.0):
            return
        try:
            payload = json.dumps({
                "heading": round(msg.data, 4)
            })
            self.mqtt.publish("/lolo/smarc/heading", payload)
            self.get_logger().info(f'Published heading: {payload}')
        except Exception as e:
            print(e)

    def callback_course(self, msg):
        if not self._should_publish('/lolo/smarc/course', 2.0):
            return
        try:
            payload = json.dumps({
                "course": round(msg.data, 4)
            })
            self.mqtt.publish("/lolo/smarc/course", payload)
            self.get_logger().info(f'Published course: {payload}')
        except Exception as e:
            print(e)

    def callback_latlon(self, msg):
        if not self._should_publish('/lolo/smarc/latlon', 2.0):
            return
        try:
            payload = json.dumps({
                "latitude":  round(msg.latitude, 8),
                "longitude": round(msg.longitude, 8),
                "altitude":  round(msg.altitude, 4)
            })
            self.mqtt.publish("/lolo/smarc/latlon", payload)
            self.get_logger().info(f'Published latlon: {payload}')
        except Exception as e:
            print(e)

    def callback_navsatfix(self, msg):
        if not self._should_publish('/lolo/standard/navsatfix', 2.0):
            return
        try:
            payload = json.dumps({
                "latitude":                 round(msg.latitude, 8),
                "longitude":                round(msg.longitude, 8),
                "altitude":                 round(msg.altitude, 4),
                "status":                   int(msg.status.status),
                "service":                  int(msg.status.service),
                "cov_latitude":             round(msg.position_covariance[0], 6),
                "cov_longitude":            round(msg.position_covariance[4], 6),
                "cov_altitude":             round(msg.position_covariance[8], 6),
                "position_covariance_type": int(msg.position_covariance_type)
            })
            self.mqtt.publish("/lolo/standard/navsatfix", payload)
            self.get_logger().info(f'Published navsatfix: {payload}')
        except Exception as e:
            print(e)

    def callback_thruster(self, msg, key):
        """Store the latest RPM for one thruster and publish all of them
        together (throttled to 1 Hz) on /lolo/extended/thrusters."""
        try:
            self._thruster_state[key] = {
                "rpm":        round(msg.rpm, 1),
                "target_rpm": round(msg.target_rpm, 1)
            }
        except Exception as e:
            print(e)
            return

        # one aggregated publish per second, regardless of which thruster fired
        if not self._should_publish('/lolo/extended/thrusters', 1.0):
            return
        try:
            payload_dict = {}
            for k, v in self._thruster_state.items():
                payload_dict[f"{k}_rpm"] = v["rpm"]
                payload_dict[f"{k}_target_rpm"] = v["target_rpm"]
            payload = json.dumps(payload_dict)
            self.mqtt.publish("/lolo/extended/thrusters", payload)
            self.get_logger().info(f'Published thrusters: {payload}')
        except Exception as e:
            print(e)

    def callback_internal_pressure(self, msg):
        if not self._should_publish('/lolo/extended/internal_pressure', 5.0):
            return
        try:
            payload = json.dumps({
                "usbl_isb":               round(msg.usbl_isb, 2),
                "thrusters_isb":          round(msg.thrusters_isb, 2),
                "vertical_thrusters_isb": round(msg.vertical_thrusters_isb, 2),
                "prevco_isb":             round(msg.prevco_isb, 2),
                "edw_isb":                round(msg.edw_isb, 2),
                "battery1_isb":           round(msg.battery1_isb, 2),
                "battery2_isb":           round(msg.battery2_isb, 2)
            })
            self.mqtt.publish("/lolo/extended/internal_pressure", payload)
            self.get_logger().info(f'Published pressure: {payload}')
        except Exception as e:
            print(e)

    def callback_internal_temperature(self, msg):
        if not self._should_publish('/lolo/extended/internal_temperature', 5.0):
            return
        try:
            payload = json.dumps({
                "captain_cpu":             msg.captain_cpu,
                "time_cpu":                msg.time_cpu,
                "captain_top":             msg.captain_top,
                "captain_eth":             msg.captain_eth,
                "captain_nuc":             msg.captain_nuc,
                "usbl_isb":                msg.usbl_isb,
                "actuator_cpu":            msg.actuator_cpu,
                "elevator_pcb":            msg.elevator_pcb,
                "elevator_motor":          msg.elevator_motor,
                "rudder_motor":            msg.rudder_motor,
                "rudder_pcb":              msg.rudder_pcb,
                "elevon_port_motor":       msg.elevon_port_motor,
                "elevon_port_pcb":         msg.elevon_port_pcb,
                "elevon_strb_motor":       msg.elevon_stbd_motor,
                "elevon_strb_pcb":         msg.elevon_stbd_pcb,
                "thruster_isb":            msg.thruster_isb,
                "port_esc":                msg.port_esc,
                "strb_esc":                msg.stbd_esc,
                "vertical_thruster_isb":   msg.vertical_thruster_isb,
                "vertical_thruster_1_esc": msg.vertical_thruster_1_esc,
                "vertical_thruster_2_esc": msg.vertical_thruster_2_esc,
                "vertical_thruster_3_esc": msg.vertical_thruster_3_esc,
                "vertical_thruster_4_esc": msg.vertical_thruster_4_esc,
                "prevco_isb":              msg.prevco_isb,
                "edw_isb":                 msg.edw_isb,
                "battery1_isb":            msg.battery1_isb,
                "battery1_temp1":          msg.battery1_temp1,
                "battery1_temp2":          msg.battery1_temp2,
                "battery1_temp3":          msg.battery1_temp3,
                "battery1_temp4":          msg.battery1_temp4,
                "battery1_temp5":          msg.battery1_temp5,
                "battery2_isb":            msg.battery2_isb,
                "battery2_temp1":          msg.battery2_temp1,
                "battery2_temp2":          msg.battery2_temp2,
                "battery2_temp3":          msg.battery2_temp3,
                "battery2_temp4":          msg.battery2_temp4,
                "battery2_temp5":          msg.battery2_temp5
            })
            self.mqtt.publish("/lolo/extended/internal_temperature", payload)
            self.get_logger().info(f'Published temperature: {payload}')
        except Exception as e:
            print(e)

    def callback_battery1(self, msg):
        if not self._should_publish('/lolo/extended/battery1', 2.0):
            return
        try:
            payload = json.dumps({
                "voltage":             round(msg.voltage, 3),
                "current":             round(msg.current, 3),
                "charge":              round(msg.charge, 3),
                "capacity":            round(msg.capacity, 3),
                "percentage":          round(msg.percentage, 4),
                "power_supply_status": int(msg.power_supply_status),
                "present":             bool(msg.present),
                "cell_voltage_min":    round(min(msg.cell_voltage), 4) if msg.cell_voltage else None,
                "cell_voltage_max":    round(max(msg.cell_voltage), 4) if msg.cell_voltage else None,
                "cell_voltage_avg":    round(sum(msg.cell_voltage)/len(msg.cell_voltage), 4) if msg.cell_voltage else None
            })
            self.mqtt.publish("/lolo/extended/battery1", payload)
            self.get_logger().info(f'Published battery1: {payload}')
        except Exception as e:
            print(e)

    def callback_battery2(self, msg):
        if not self._should_publish('/lolo/extended/battery2', 2.0):
            return
        try:
            payload = json.dumps({
                "voltage":             round(msg.voltage, 3),
                "current":             round(msg.current, 3),
                "charge":              round(msg.charge, 3),
                "capacity":            round(msg.capacity, 3),
                "percentage":          round(msg.percentage, 4),
                "power_supply_status": int(msg.power_supply_status),
                "present":             bool(msg.present),
                "cell_voltage_min":    round(min(msg.cell_voltage), 4) if msg.cell_voltage else None,
                "cell_voltage_max":    round(max(msg.cell_voltage), 4) if msg.cell_voltage else None,
                "cell_voltage_avg":    round(sum(msg.cell_voltage)/len(msg.cell_voltage), 4) if msg.cell_voltage else None
            })
            self.mqtt.publish("/lolo/extended/battery2", payload)
            self.get_logger().info(f'Published battery2: {payload}')
        except Exception as e:
            print(e)

    def callback_status(self, msg):
        if not self._should_publish('/lolo/extended/status', 2.0):
            return
        try:
            payload = json.dumps({
                # Power
                "voltage":                    round(msg.voltage, 2),
                "current":                    round(msg.current, 2),
                "rc_signal":                  int(msg.rc_signal),
                # State
                "emergency_status":           msg.emergency_status,
                "control_mode":               msg.control_mode,
                "control_source":             msg.control_source,
                # Leaks
                "leak_captain":               bool(msg.captain_leak),
                "leak_esc":                   bool(msg.esc_leak),
                "leak_prevco":                bool(msg.prevco_leak),
                "leak_edw":                   bool(msg.edw_leak),
                "leak_battery1":              bool(msg.battery1_leak),
                "leak_battery2":              bool(msg.battery2_leak),
                # System statuses
                "sys_time":                   bool(msg.time_status),
                "sys_trigger":                bool(msg.trigger_status),
                "sys_actuators":              bool(msg.actuators_status),
                "sys_thrusters":              bool(msg.thrusters_status),
                "sys_vertical_thrusters":     bool(msg.vertical_thrusters_status),
                "sys_usbl":                   bool(msg.usbl_status),
                "sys_scientist":              bool(msg.scientist_status),
                "sys_edw":                    bool(msg.edw_status),
                "sys_battery1":               bool(msg.battery1_status),
                "sys_battery2":               bool(msg.battery2_status),
                # Outputs
                "out_aux":                    bool(msg.aux_output),
                "out_servo1":                 bool(msg.servo1_output),
                "out_servo2":                 bool(msg.servo2_output),
                "out_lumen":                  bool(msg.lumen_output),
                "out_mbes":                   bool(msg.mbes_output),
                # Thrusters
                "thrusters_enabled":          bool(msg.thrusters_enabled),
                "vertical_thrusters_enabled": bool(msg.vertical_thrusters_enabled),
                # EDW
                "edw_armed":                  bool(msg.edw_armed),
                "edw_timer_armed":            bool(msg.edw_timer_armed),
                "edw_timer_time_left":        int(msg.edw_timer_time_left)
            })
            self.mqtt.publish("/lolo/extended/status", payload)
            self.get_logger().info(f'Published status: {payload}')
        except Exception as e:
            print(e)

    def mqtt_on_connect(self, client, userdata, flags, reason_code):
        print(f"Connected with result code {reason_code}")

    def mqtt_on_message(self, client, userdata, msg):
        pass


def main(args=None):
    rclpy.init(args=args)
    mqttc = mqtt.Client()
    translator = Translator_node(mqttc)
    mqttc.on_connect = translator.mqtt_on_connect
    mqttc.on_message = translator.mqtt_on_message

    # Publish to Lolo's own local broker
    mqttc.connect("localhost", 1883, 60)

    mqttc.loop_start()
    executor = MultiThreadedExecutor()
    executor.add_node(translator)
    executor.spin()
    translator.destroy_node()
    mqttc.loop_stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
