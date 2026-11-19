#include "controller.h"
#include <chrono>
#include <stdio.h>
#include "../lolo_message_id.h"

#include "lolo_msgs/msg/topics.hpp"
#include "smarc_msgs/msg/topics.hpp"

using namespace std::chrono_literals;

Controller::Controller(CaptainInterFace* _lolo, rclcpp::Node* _rcl_node) {
    lolo = _lolo;
    rcl_node = _rcl_node;
};

void Controller::setup() {
    discovery_timer_ = rcl_node->create_wall_timer(500ms, std::bind(&Controller::send_discovery, this));

    //Create subscriptions
    //heartbeat_sub = rcl_node->create_subscription<std_msgs::msg::Empty>(lolo_msgs::msg::Topics::HEARTBEAT_TOPIC, 1, std::bind(&Controller::heartbeat_callback, this, std::placeholders::_1));
    abort_sub = rcl_node->create_subscription<std_msgs::msg::Empty>(lolo_msgs::msg::Topics::LOLO_ABORT, 1, std::bind(&Controller::abort_callback, this, std::placeholders::_1));
    
    rudder_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::RUDDER_CMD, 1, std::bind(&Controller::rudder_callback, this, std::placeholders::_1));
    elevator_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::ELEVATOR_CMD, 1, std::bind(&Controller::elevator_callback, this, std::placeholders::_1));
    elevon_port_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::ELEVON_PORT_CMD, 1, std::bind(&Controller::elevon_port_callback, this, std::placeholders::_1));
    elevon_stbd_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::ELEVON_STBD_CMD, 1, std::bind(&Controller::elevon_stbd_callback, this, std::placeholders::_1));
    
    thruster_port_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::THRUSTER_PORT_CMD, 1, std::bind(&Controller::thruster_port_callback, this, std::placeholders::_1));
    thruster_stbd_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::THRUSTER_STBD_CMD, 1, std::bind(&Controller::thruster_stbd_callback, this, std::placeholders::_1));
    vertical_thruster_1_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::VERTICAL_THRUSTER_FRONT_PORT_CMD, 1, std::bind(&Controller::vertical_thruster_1_callback, this, std::placeholders::_1));
    vertical_thruster_2_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::VERTICAL_THRUSTER_FRONT_STBD_CMD, 1, std::bind(&Controller::vertical_thruster_2_callback, this, std::placeholders::_1));
    vertical_thruster_3_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::VERTICAL_THRUSTER_BACK_PORT_CMD, 1, std::bind(&Controller::vertical_thruster_3_callback, this, std::placeholders::_1));
    vertical_thruster_4_cmd_sub = rcl_node->create_subscription<std_msgs::msg::Float32>(lolo_msgs::msg::Topics::VERTICAL_THRUSTER_BACK_STBD_CMD, 1, std::bind(&Controller::vertical_thruster_4_callback, this, std::placeholders::_1));
    
    menu_sub = rcl_node->create_subscription<std_msgs::msg::String>("debug/menu_in", 1, std::bind(&Controller::menu_callback, this, std::placeholders::_1));
    
    usbl_sub = rcl_node->create_subscription<std_msgs::msg::String>(lolo_msgs::msg::Topics::USBL_TRANSMIT_TOPIC, 10, std::bind(&Controller::usbl_callback, this, std::placeholders::_1));
    
    satelite_sub = rcl_node->create_subscription<std_msgs::msg::String>(lolo_msgs::msg::Topics::SATELITE_RECEIVED_TOPIC, 1, std::bind(&Controller::satelite_callback, this, std::placeholders::_1));

    settings_sub = rcl_node->create_subscription<diagnostic_msgs::msg::KeyValue>(lolo_msgs::msg::Topics::EXTENDED_SETTINGS_TOPIC, 10, std::bind(&Controller::settings_callback, this, std::placeholders::_1));
}

void Controller::send_discovery() {
    printf("Sending discovery package\n");
    //Send something to lolo so it can get the ip of the interface computer
    lolo->new_package(0);
    lolo->send_package();
}

//ROS2 callback functions

void Controller::abort_callback(const std_msgs::msg::Empty::SharedPtr _msg) {
    lolo->new_package(SC_ABORT); // Tell captain to go into emergency mode
    lolo->send_package();
    std::cout << _msg << std::endl;
}

void Controller::heartbeat_callback(const std_msgs::msg::Empty::SharedPtr _msg) {
    lolo->new_package(SC_HEARTBEAT); // Heartbeat message
    lolo->send_package();
    std::cout << _msg << std::endl;
}

void Controller::drop_weight_callback(const std_msgs::msg::Empty::SharedPtr _msg) {
    lolo->new_package(SC_DROP_WEIGHT); // Tell captain to go into emergency mode
    lolo->add_byte(1);
    lolo->send_package();
    std::cout << _msg << std::endl;
}

void Controller::rudder_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    float angle = _msg->data;
    lolo->new_package(SC_SET_RUDDER);
    lolo->add_float(angle);
    lolo->send_package();
}

void Controller::elevator_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    float angle = _msg->data;
    lolo->new_package(SC_SET_ELEVATOR);
    lolo->add_float(angle);
    lolo->send_package();
}

void Controller::elevon_port_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    float angle = _msg->data;
    lolo->new_package(SC_SET_ELEVON_PORT);
    lolo->add_float(angle);
    lolo->send_package();
}

void Controller::elevon_stbd_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    float angle = _msg->data;
    lolo->new_package(SC_SET_ELEVON_STBD);
    lolo->add_float(angle);
    lolo->send_package();
}

void Controller::thruster_port_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_THRUSTER_PORT);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::thruster_stbd_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_THRUSTER_STBD);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::vertical_thruster_1_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_VTHRUSTER_1);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::vertical_thruster_2_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_VTHRUSTER_2);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::vertical_thruster_3_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_VTHRUSTER_3);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::vertical_thruster_4_callback(const std_msgs::msg::Float32::SharedPtr _msg) {
    lolo->new_package(SC_SET_VTHRUSTER_4);
    lolo->add_float(_msg->data);
    lolo->send_package();
}

void Controller::menu_callback(const std_msgs::msg::String::SharedPtr _msg) {
    lolo->new_package(SC_MENUSTREAM);
    std::string s = _msg->data;
    uint8_t bytes = std::min(200, (int) s.size());
    lolo->add_byte(bytes);
    for(int i=0;i<bytes;i++) {
      lolo->add_byte(s[i]);
    }
    lolo->send_package();
}

void Controller::usbl_callback(const std_msgs::msg::String::SharedPtr _msg) {
    lolo->new_package(SC_USBL_TRANSMIT);
    std::string s = _msg->data;
    uint8_t bytes = std::min(120, (int) s.size());
    lolo->add_byte(bytes);
    for(int i=0;i<bytes;i++) {
      lolo->add_byte(s[i]);
    }
    lolo->send_package();
}

void Controller::satelite_callback(const std_msgs::msg::String::SharedPtr _msg) {
    lolo->new_package(SC_SATELITE_TRANSMIT);
    std::string s = _msg->data;
    uint8_t bytes = std::min(120, (int) s.size());
    lolo->add_byte(bytes);
    for(int i=0;i<bytes;i++) {
      lolo->add_byte(s[i]);
    }
    lolo->send_package();
}

void Controller::settings_callback(const diagnostic_msgs::msg::KeyValue::SharedPtr _msg) {
    //TODO settings stuff
    std::printf("Received change settings message. Key=%s, value=%s\r\n", _msg->key.c_str(), _msg->value.c_str());

    //Check what the key corresponds to
    if(_msg->key.compare("control_source") == 0) {
        std::printf("Key is control source\n");
        if(_msg->value.compare("MANUAL_CONTROL") == 0) {
            std::printf("Setting control source to manual control\r\n");
            lolo->new_package(SC_SETTINGS);
            lolo->add_byte(SC_SETTINGS_CONTROL_SOURCE_MANUAL_CONTROL);
            lolo->send_package();
        }
        else if(_msg->value.compare("ONBOARD_CONTROL") == 0) {
            std::printf("Setting control source to onboard control\r\n");
            lolo->new_package(SC_SETTINGS);
            lolo->add_byte(SC_SETTINGS_CONTROL_SOURCE_ONBOARD_CONTROL);
            lolo->send_package();
        }
        else if(_msg->value.compare("EXTERNAL_CONRTOL") == 0) {
            std::printf("Setting control source to external control\r\n");
            lolo->new_package(SC_SETTINGS);
            lolo->add_byte(SC_SETTINGS_CONTROL_SOURCE_EXTERNAL_CONRTOL);
            lolo->send_package();
        }
        else if(_msg->value.compare("EMERGENCY_CONTROL") == 0) {
            std::printf("Setting control source to emergency control\r\n");
            lolo->new_package(SC_SETTINGS);
            lolo->add_byte(SC_SETTINGS_CONTROL_SOURCE_EMERGENCY_CONTROL);
            lolo->send_package();
        }
        else {
            std::printf("Unknown control source\r\n");
        }
    }
    else if(_msg->key.compare("control_mode") == 0) {
        std::printf("Key is control mode\n");
        if(_msg->value.compare("STANDBY") == 0) {
            std::printf("Setting control mode to standby\r\n");
            lolo->new_package(SC_SETTINGS);
            lolo->add_byte(SC_SETTINGS_CONTROL_MODE_STANDBY);
            lolo->send_package();
        }
        else if(_msg->value.compare("DISARMED") == 0) {
            std::printf("Setting control mode to disarmed\r\n");
            lolo->new_package(SC_SETTINGS);
            lolo->add_byte(SC_SETTINGS_CONTROL_MODE_DISARMED);
            lolo->send_package();
        }
        else if(_msg->value.compare("ARMED") == 0) {
            std::printf("Setting control mode to armed\r\n");
            lolo->new_package(SC_SETTINGS);
            lolo->add_byte(SC_SETTINGS_CONTROL_MODE_ARMED);
            lolo->send_package();
        }
        else {
            std::printf("Unknown control mode\r\n");
        }
    }
    else if(_msg->key.compare("thrusters_enabled") == 0) {
        std::printf("Key is thrusters\n");
        if(_msg->value.compare("0") == 0) {
            std::printf("Disabling thrusters\r\n");
            lolo->new_package(SC_SETTINGS);
            lolo->add_byte(SC_SETTINGS_THRUSTERS_DISABLED);
            lolo->send_package();
        }
        if(_msg->value.compare("1") == 0) {
            std::printf("Enabling thrusters\r\n");
            lolo->new_package(SC_SETTINGS);
            lolo->add_byte(SC_SETTINGS_THRUSTERS_ENABLED);
            lolo->send_package();
        }
    }
    else if(_msg->key.compare("vertical_thrusters_enabled") == 0) {
        std::printf("Key is vertical thrusters\n");
    }

}
