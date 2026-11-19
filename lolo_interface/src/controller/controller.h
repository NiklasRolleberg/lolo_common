#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <udpinterface/CaptainInterFace.h>
#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"

//Controller class sends ros2 commands to lolo
class Controller {

CaptainInterFace* lolo;
rclcpp::Node* rcl_node;
public:
    Controller(CaptainInterFace* _lolo, rclcpp::Node* _rcl_node);

    void setup();

private:
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    void send_discovery();

    //Subscribers
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr heartbeat_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr abort_sub;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr drop_weight_sub;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rudder_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr elevator_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr elevon_port_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr elevon_stbd_cmd_sub;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thruster_port_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr thruster_stbd_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vertical_thruster_1_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vertical_thruster_2_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vertical_thruster_3_cmd_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr vertical_thruster_4_cmd_sub;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr menu_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr usbl_sub;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr satelite_sub;

    //Settings Subscriber
    rclcpp::Subscription<diagnostic_msgs::msg::KeyValue>::SharedPtr settings_sub;

    //INS / position?
    
    //Callback functions
    void abort_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void heartbeat_callback(const std_msgs::msg::Empty::SharedPtr msg);
    void drop_weight_callback(const std_msgs::msg::Empty::SharedPtr msg);

    void rudder_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void elevator_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void elevon_port_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void elevon_stbd_callback(const std_msgs::msg::Float32::SharedPtr msg);

    void thruster_port_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void thruster_stbd_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void vertical_thruster_1_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void vertical_thruster_2_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void vertical_thruster_3_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void vertical_thruster_4_callback(const std_msgs::msg::Float32::SharedPtr msg);

    void menu_callback(const std_msgs::msg::String::SharedPtr msg);

    void usbl_callback(const std_msgs::msg::String::SharedPtr msg);

    void satelite_callback(const std_msgs::msg::String::SharedPtr msg);

    void settings_callback(const diagnostic_msgs::msg::KeyValue::SharedPtr msg);
    
    
};

#endif