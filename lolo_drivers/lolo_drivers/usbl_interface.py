import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Char, String
from std_msgs.msg import Empty
from std_msgs.msg import Float32
from geographic_msgs.msg import GeoPoint
from lolo_msgs.msg import UsblTopsideCorrection
from lolo_msgs.msg import CustomCommand
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time


from lolo_msgs.msg import Topics as LoloTopics
from smarc_msgs.msg import Topics as SmarcTopics

# For UTC time.
import time

# Useful USBL AT commands.
DROP_MSG = "+++ATZ4\r"
SET_PWR_LOW = "+++AT!L3\r"

# Device address for the local (Lolo) acoustic modem.
LOCAL_ADDRESS = 1
# Device address for the remote (topside) acoustic modem.
REMOTE_ADDRESS = 2

# Device address for the local (Lolo) acoustic modem.
LOCAL_ADDRESS = 2
# Device address for the remote (topside) acoustic modem.
REMOTE_ADDRESS = 1

# Newline in ASCII.
NEWLINE = 10

# TODO: Max length for the command buffer, assuming 1000 chars is too long.
CMD_MAX_LEN = 500

# In air depth threshold in [m]
IN_AIR_DEPTH = 1.0

# Length of the checksum.
CHECKSUM_LEN = 3

class UsblInterface:

    def __init__(self, node):

        self.node = node
        self.log = self.node.get_logger()
        self.command = ""

        # Important flags.
        self.usbl_in_air = True # Will assume it's always starting on air.

        self.cur_lat = None
        self.cur_lon = None
        self.cur_depth = None

        self.node.declare_parameter("usbl_topside_frame", "/usbl_topside_enu")
        self.topside_frame_id = self.node.get_parameter("usbl_topside_frame").value

        # Publisher for the transmit message.
        self.transmit_pub = self.node.create_publisher(String, LoloTopics.USBL_TRANSMIT_TOPIC,
                                                       1)
        # Publisher for the abort message.
        self.abort_pub = self.node.create_publisher(Empty, LoloTopics.LOLO_ABORT,10)
        # Publisher for the USBL correction sent by the topside unit.
        self.usbl_correction_pub = self.node.create_publisher(UsblTopsideCorrection,
                                                              LoloTopics.USBL_CORRECTION_TOPIC,
                                                              10)
        # Publisher for a custom command from the topside unit.
        self.custom_cmd_pub = self.node.create_publisher(CustomCommand,
                                                         LoloTopics.USBL_CUSTOM_CMD_TOPIC,
                                                         10)
        # Publisher for the measured relative position from the topside unit.
        self.usbl_relative_pub = self.node.create_publisher(PoseStamped,
                                                            LoloTopics.USBL_RELATIVE_TOPIC,
                                                            10)

        # ------ STATUS SUBSCRIBERS ----------
        # TODO: Subscribe to a bunch (or a custom) status topics that include relevant
        # info to send to the topside modem.
        # ------------------------- ----------

       # Subscribe from the get-go.
        self.node.create_subscription(Char, LoloTopics.USBL_RECEIVED_CHR_TOPIC,
                                    self.usbl_callback, 1000)
        self.node.create_subscription(GeoPoint, SmarcTopics.POS_LATLON_TOPIC,
                                    self.position_callback, 10)
        self.node.create_subscription(Float32, SmarcTopics.DEPTH_TOPIC,
                                    self.depth_callback, 10)

        # FIXME: self.configure_modem()

    def usbl_callback(self, msg):
        """
        Incrementally build the inbound command until a newline appears.
        """
        self.command += chr(msg.data)

        # If we get a newline, we assume we've got the whole command.
        if self.command[-1] == chr(NEWLINE):

            self.log.info("Got command {0} from topside modem.".format(self.command))
            success = self.usbl_menu(self.command[:-1])
            if not success:
                self.log.warning("Did not understand command {0}.".format(self.command))
            # Reset the command buffer.
            self.command = ""

        if len(self.command) > CMD_MAX_LEN:
            self.log.warning("Newline hasn't been received, dropping buffer.")
            self.command = ""

    def position_callback(self, msg):
        """
        Just store the lat/lon.
        """
        self.cur_lat = round(msg.latitude,6)
        self.cur_lon = round(msg.longitude,6)

    def depth_callback(self, msg):
        """
        Store depth, trigger usbl_in_air flag.
        """
        self.cur_depth = round(msg.data,6)

        # Trigger usbl_in_air flag if lolo is at the surface.
        if self.cur_depth >= IN_AIR_DEPTH:
            self.usbl_in_air = False
        else:
            self.usbl_in_air = True

    def configure_modem(self):
        # FIXME: We're not using it ATM, we have to test it more,
        # last time it broke the communication between devices...
        """
        Configure the communication modem to comply with the necessary
        settings for burst message communication (has to be the same as
        the topside unit).
        """
        rate = self.node.create_rate(1)
        # Set the transmission level to the lowest to be safe.
        self.send_for_transmission("+++AT!L3", is_command=True)
        self.log.info("(UsblInterface) Configuring transmission to lowest power.")
        rate.sleep()

        # Set the receiving gain to normal.
        self.send_for_transmission("+++AT!G0", is_command=True)
        self.log.info("(UsblInterface) Setting reciever gain to normal.")
        rate.sleep()

        # Set the local and remote device addresses for lolo (1) and the topside (2).
        self.send_for_transmission("+++AT!AL2", is_command=True)
        self.log.info("(UsblInterface) Setting local address to 2.")
        rate.sleep()
        self.send_for_transmission("+++AT!AR1", is_command=True)
        self.log.info("(UsblInterface) Setting remote address to 1.")
        rate.sleep()

        # Set the packet transmission and reception time. Setting it to max (1200ms).
        self.send_for_transmission("+++AT!ZP1200", is_command=True)
        self.log.info("(UsblInterface) Setting max packet transmission time to MAX (1200ms).")
        rate.sleep()

        rate.destroy()

    def usbl_menu(self, msg):

        """
        Use the input command to do something useful.
        You can edit this function to include more features.

        msg:
            type: string
            format: "CMD x x x x ..."
        """
        arg_list = [m for m in msg.split(" ")]
        command = arg_list[0]

        if command.lower() == 'abort':
            self.log.warning("(UsblInterface) ABORT ABORT ABORT!!")
            self.abort_pub.publish(Empty())
            success = True
        elif command.lower() == 'pos':
            self.log.info("(UsblInterface) Position requested by topside, sending...")
            self.relay_position()
            success = True
        elif command.lower() == 'stat':
            self.log.info("(UsblInterface) Status requested by topside, sending...")
            self.relay_status()
            success = True
        elif command.lower() == 'cor':
            self.log.info("(UsblInterface) Receiving USBL correction...")
            self.publish_usbl_correction(msg)
            success = True
        elif command.lower() == 'runcmd':
            self.log.info("(UsblInterface) Getting custom command from topside...")
            self.publish_custom_cmd(msg)
            success = True
        elif command.lower() == 'rel':
            self.log.info("(UsblInterface) Receiving a relative position from topside...")
            self.publish_relative_position(msg)
            success = True
        else:
            success = False

        return success

    def relay_status(self):
        """
        TODO: Relay the most recent status update.
        """
        msg = "STAT " + str(self.cur_lat) + " " + str(self.cur_lon) + " " + str(self.cur_depth)
        self.log.info("(UsblInterface) Relaying status to topside: {0}".format(msg))

        self.send_for_transmission(msg)

    def relay_position(self):
        """
        Relay the most recent position of LoLo in lat/lon [deg] and depth [m].
        """

        #rospy.sleep(10)
        msg = "POS " + str(self.cur_lat) + " " + str(self.cur_lon) + " " + str(self.cur_depth)
        self.log.info("(UsblInterface) Relaying position to topside: {0}".format(msg))

        self.send_for_transmission(msg)

    def publish_usbl_correction(self, msg):
        """
        Publish the USBL's position and measured rangeso the INS can
        use it for correction.

        msg:
            type: string
            format: "COR usbl_lat usbl_lon usbl_range stamp"
        """
        # Split string by the whitespace between each variable.
        try:
            cmd, lat, lon, meas_range, stamp = [m for m in msg.split(" ")]
        except:
            self.log.error("(UsblInterface) Too many values to unpack, message is wrong.")
            self.command = ""
            return

        # Build message and publish.
        usbl_correction = UsblTopsideCorrection()
        usbl_correction.lat = float(lat)
        usbl_correction.lon = float(lon)
        usbl_correction.range = float(meas_range)
        usbl_correction.stamp = float(stamp)
        self.usbl_correction_pub.publish(usbl_correction)

    def publish_custom_cmd(self, msg):
        """
        Publish the custom commands as a CustomCommand msg
        for other nodes to do something with it.

        msg:
            type: string
            format: "cmd customcmd sent_stamp checksum"
        """
        # Split string by the whitespace between each variable.
        command = [m for m in msg.split(" ")]

        # Checksum will be the sum of the bytes in the msg without the checksum.
        bytelist = [ord(c) for c in msg[:-CHECKSUM_LEN]]
        checksum = hex(sum(bytearray(bytelist)) % 256)[2:]
        if command[-1] != checksum:
            self.log.error("(UsblInterface) Custom cmd checksum failed! Ignoring.")
            self.log.error("(UsblInterface) Expected {0} and got {1}.".format(command[-1], checksum))
            return

        custom_msg = CustomCommand()
        custom_msg.cmd = command[1]
        custom_msg.sent_stamp = int(command[2])
        # Stamp it with UTC time in [s].
        custom_msg.recv_stamp = int(time.time())

        self.custom_cmd_pub.publish(custom_msg)

    def publish_relative_position(self, msg):
        """
        Publish the relayed relative position of Lolo with respect to the
        topside unit, in ENU convention.

        msg:
            type: string
            format: "REL East North Up"
        """
        # Split string by the whitespace between each variable.
        try:
            cmd, east, north, up = [m for m in msg.split(" ")]
        except:
            self.log.error("(UsblInterface) Too many values to unpack, message is wrong.")
            self.command = ""
            return

        # TODO: we would like to eventually send a timestamp from the topside.
        # stamp_msg = Time()
        # stamp = float(stamp)
        # stamp_msg.sec = int(stamp)
        # stamp_msg.nanosec = int((stamp % 1) * 1e9)
        pos_msg = PoseStamped()
        # TODO: will the fact that the stamp is in the past affect ros in a way?
        pos_msg.header.stamp = self.node.get_clock().now().to_msg()
        pos_msg.header.frame_id = self.topside_frame_id
        pos_msg.pose.position.x = float(east)
        pos_msg.pose.position.y = float(north)
        pos_msg.pose.position.z = float(up)
        self.usbl_relative_pub.publish(pos_msg)

    def send_for_transmission(self, msg, is_command=False):
        """
        Send the input msg to the topside unit. Checks if lolo is at the
        surface before transmitting anything.
        """
        # Do not transmit if lolo is at the surface.
        if self.usbl_in_air and not is_command:
            self.log.warning("(UsblInterface) USBL in air! Won't transmit to topside!")
            return

        # Append a carriage return at the end of the msg.
        msg += "\r\n"
        string_msg = String()
        string_msg.data = msg
        self.transmit_pub.publish(string_msg)

    def drop_message(self):
        """
        Send an ATZ4 command to the transducer to drop messages.
        """
        if self.usbl_in_air:
            self.log.warning("(UsblInterface) In air, dropping messages...")
            string_msg = String()
            string_msg.data = DROP_MSG
            self.transmit_pub.publish(string_msg)


def main(args=None):
    rclpy.init(args=args)
    node_name = "lolo_usbl_interface"
    node = rclpy.node.Node(node_name)
    # Only instantiating the class should be enough.
    interface = UsblInterface(node)
    node.create_timer(5.0, interface.drop_message)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
