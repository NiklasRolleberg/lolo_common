#ifndef LOLO_INTERFACE_VIEW_H
#define LOLO_INTERFACE_VIEW_H

#include <udpinterface/CaptainInterFace.h>
#include "rclcpp/rclcpp.hpp"

//Message types
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/char.hpp"
#include "std_msgs/msg/empty.hpp"
#include "lolo_msgs/msg/status.hpp"
#include "lolo_msgs/msg/volz_servo.hpp"
#include "lolo_msgs/msg/vesc_feedback.hpp"
#include "lolo_msgs/msg/pressures.hpp"
#include "lolo_msgs/msg/temperatures.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

#include "lolo_msgs/msg/fuel_celldata_fcu.hpp"
#include "lolo_msgs/msg/fuel_celldata_sys.hpp"

//View class publishes data from lolo to ros2 
class View 
{
    CaptainInterFace* lolo;
    rclcpp::Node* rcl_node;

    //////////////////////////////Publishers////////////////////////

    //Text menu
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr menu_pub;
    
    //LEAK
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr leak_pub;
    
    //Status
    rclcpp::Publisher<lolo_msgs::msg::Status>::SharedPtr status_pub;

    //Temperature
    rclcpp::Publisher<lolo_msgs::msg::Temperatures>::SharedPtr temperature_pub;

    //Pressure
    rclcpp::Publisher<lolo_msgs::msg::Pressures>::SharedPtr pressure_pub;

    //Control surfaces
    rclcpp::Publisher<lolo_msgs::msg::VolzServo>::SharedPtr rudder_pub;
    rclcpp::Publisher<lolo_msgs::msg::VolzServo>::SharedPtr elevator_pub;
    rclcpp::Publisher<lolo_msgs::msg::VolzServo>::SharedPtr elevon_port_pub;
    rclcpp::Publisher<lolo_msgs::msg::VolzServo>::SharedPtr elevon_strb_pub;

    //Thrusters
    rclcpp::Publisher<lolo_msgs::msg::VescFeedback>::SharedPtr thruster_port_pub;
    rclcpp::Publisher<lolo_msgs::msg::VescFeedback>::SharedPtr thruster_strb_pub;
    rclcpp::Publisher<lolo_msgs::msg::VescFeedback>::SharedPtr vertical_thruster_1_pub;
    rclcpp::Publisher<lolo_msgs::msg::VescFeedback>::SharedPtr vertical_thruster_2_pub;
    rclcpp::Publisher<lolo_msgs::msg::VescFeedback>::SharedPtr vertical_thruster_3_pub;
    rclcpp::Publisher<lolo_msgs::msg::VescFeedback>::SharedPtr vertical_thruster_4_pub;

    //Battery
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery1_pub;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery2_pub;

    //Usbl
    rclcpp::Publisher<std_msgs::msg::Char>::SharedPtr usbl_pub;

    //Satelite
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr satelite_pub;

    //Fuelcell
    rclcpp::Publisher<lolo_msgs::msg::FuelCelldataFCU>::SharedPtr fc_FCU_pub;
    rclcpp::Publisher<lolo_msgs::msg::FuelCelldataSYS>::SharedPtr fc_SYS_pub;

    //callback functions
    void lolo_callback_LEAK();
    void lolo_callback_STATUS();
    void lolo_callback_RUDDER();
    void lolo_callback_ELEVATOR();
    void lolo_callback_ELEVON_PORT();
    void lolo_callback_ELEVON_STRB();
    void lolo_callback_THRUSTER_PORT();
    void lolo_callback_THRUSTER_STRB();
    void lolo_callback_VERTICAL_THRUSTER(int thruster_id);
    void lolo_callback_BATTERY(int id);
    void lolo_callback_MENUSTREAM();
    void lolo_callback_MISSIONLOG();
    void lolo_callback_USBL_RECEIVED();
    void lolo_callback_SATELITE_RECEIVED();
    void lolo_callback_TEMP();
    void lolo_callback_BARO();
    void lolo_callback_FC_FCU();
    void lolo_callback_FC_SYS();

public:
    View(CaptainInterFace* _lolo, rclcpp::Node* _rcl_node);

    void setup();

    void data_callback();

};

#endif