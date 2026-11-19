import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from lolo_msgs.msg import Topics
from lolo_msgs.msg import VolzServo
from lolo_msgs.msg import VescFeedback
from std_msgs.msg import Float32

class Translator_node(Node):

    def __init__(self):
        super().__init__('extended_interface_translator')

        #Elevon port fb
        self.pub_elevon_port_fb = self.create_publisher(Float32, Topics.ELEVON_PORT_FB, 2)
        self.sub_elevon_port_fb = self.create_subscription(VolzServo,Topics.EXTENDED_PORT_ELEVON_FB_TOPIC,self.callback_elevon_port_fb,1)

        #Elevon stbd fb
        self.pub_elevon_stbd_fb = self.create_publisher(Float32, Topics.ELEVON_STBD_FB, 2)
        self.sub_elevon_stbd_fb = self.create_subscription(VolzServo,Topics.EXTENDED_STBD_ELEVON_FB_TOPIC,self.callback_elevon_stbd_fb,1)

        #Elevator
        self.pub_elevator_fb = self.create_publisher(Float32, Topics.ELEVATOR_FB, 2)
        self.sub_elevator_fb = self.create_subscription(VolzServo,Topics.EXTENDED_ELEVATOR_FB_TOPIC,self.callback_elevator_fb,1)

        #Rudder
        self.pub_rudder_fb = self.create_publisher(Float32, Topics.RUDDER_FB, 2)
        self.sub_rudder_fb = self.create_subscription(VolzServo,Topics.EXTENDED_RUDDER_FB_TOPIC,self.callback_rudder_fb,1)

        #port thruster
        self.pub_thruster_port_fb = self.create_publisher(Float32, Topics.THRUSTER_PORT_FB, 2)
        self.sub_thruster_port_fb = self.create_subscription(VescFeedback,Topics.EXTENDED_PORT_THRUSTER_FB_TOPIC,self.callback_thruster_port_fb,1)

        #port thruster
        self.pub_thruster_stbd_fb = self.create_publisher(Float32, Topics.THRUSTER_STBD_FB, 2)
        self.sub_thruster_stbd_fb = self.create_subscription(VescFeedback,Topics.EXTENDED_STBD_THRUSTER_FB_TOPIC,self.callback_thruster_stbd_fb,1)

        #Vertical thrusters
        self.pub_vertical_thruster_1_fb = self.create_publisher(Float32, Topics.VERTICAL_THRUSTER_FRONT_PORT_FB, 2)
        self.pub_vertical_thruster_2_fb = self.create_publisher(Float32, Topics.VERTICAL_THRUSTER_FRONT_STBD_FB, 2)
        self.pub_vertical_thruster_3_fb = self.create_publisher(Float32, Topics.VERTICAL_THRUSTER_BACK_PORT_FB, 2)
        self.pub_vertical_thruster_4_fb = self.create_publisher(Float32, Topics.VERTICAL_THRUSTER_BACK_STBD_FB, 2)

        self.sub_vertical_thruster_1_fb = self.create_subscription(VescFeedback,Topics.EXTENDED_VERTICAL_THRUSTER_FRONT_PORT_FB_TOPIC,self.callback_vertical_thruster_1_fb,1)
        self.sub_vertical_thruster_2_fb = self.create_subscription(VescFeedback,Topics.EXTENDED_VERTICAL_THRUSTER_FRONT_STBD_FB_TOPIC,self.callback_vertical_thruster_2_fb,1)
        self.sub_vertical_thruster_3_fb = self.create_subscription(VescFeedback,Topics.EXTENDED_VERTICAL_THRUSTER_BACK_PORT_FB_TOPIC,self.callback_vertical_thruster_3_fb,1)
        self.sub_vertical_thruster_4_fb = self.create_subscription(VescFeedback,Topics.EXTENDED_VERTICAL_THRUSTER_BACK_STBD_FB_TOPIC,self.callback_vertical_thruster_4_fb,1)


    def callback_elevon_port_fb(self, msg):
        fb_angle = Float32()
        fb_angle.data = msg.angle
        self.pub_elevon_port_fb.publish(fb_angle)
        #print(str(msg))

    def callback_elevon_stbd_fb(self, msg):
        fb_angle = Float32()
        fb_angle.data = msg.angle
        self.pub_elevon_stbd_fb.publish(fb_angle)
        #print(str(msg))
    
    def callback_elevator_fb(self, msg):
        fb_angle = Float32()
        fb_angle.data = msg.angle
        self.pub_elevator_fb.publish(fb_angle)
        #print(str(msg))

    def callback_rudder_fb(self, msg):
        fb_angle = Float32()
        fb_angle.data = msg.angle
        self.pub_rudder_fb.publish(fb_angle)
        #print(str(msg))

    def callback_thruster_port_fb(self, msg):
        fb_rpm = Float32()
        fb_rpm.data = msg.rpm
        self.pub_thruster_port_fb.publish(fb_rpm)
        #rint(str(msg))

    def callback_thruster_stbd_fb(self, msg):
        fb_rpm = Float32()
        fb_rpm.data = msg.rpm
        self.pub_thruster_stbd_fb.publish(fb_rpm)
        #print(str(msg))

    def callback_vertical_thruster_1_fb(self,msg):
        fb_rpm = Float32()
        fb_rpm.data = msg.rpm
        self.pub_vertical_thruster_1_fb.publish(fb_rpm)
        #print(str(msg))

    def callback_vertical_thruster_2_fb(self,msg):
        fb_rpm = Float32()
        fb_rpm.data = msg.rpm
        self.pub_vertical_thruster_2_fb.publish(fb_rpm)
        #print(str(msg))

    def callback_vertical_thruster_3_fb(self,msg):
        fb_rpm = Float32()
        fb_rpm.data = msg.rpm
        self.pub_vertical_thruster_3_fb.publish(fb_rpm)
        #print(str(msg))

    def callback_vertical_thruster_4_fb(self,msg):
        fb_rpm = Float32()
        fb_rpm.data = msg.rpm
        self.pub_vertical_thruster_4_fb.publish(fb_rpm)
        #print(str(msg))



def main(args=None):
    rclpy.init(args=args)
    translator = Translator_node()

    executor = MultiThreadedExecutor()
    executor.add_node(translator)
    executor.spin()
    #rclpy.spin(translator)

    translator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
