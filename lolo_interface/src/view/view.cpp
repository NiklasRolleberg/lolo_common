#include "view.h"
#include <stdio.h>
#include "../lolo_message_id.h"

#include "lolo_msgs/msg/topics.hpp"

#include <MiniCapHelpers/Split_Captain_struct.h>
//#include <MiniCapHelpers/Captain_struct.h>

View::View(CaptainInterFace* _lolo, rclcpp::Node* _rcl_node) {
    lolo = _lolo;
    rcl_node=_rcl_node;
};

void View::setup() 
{
    //Menu pub
    menu_pub = rcl_node->create_publisher<std_msgs::msg::String>("/lolo/debug/menu_out", 1);

    //leak_pub
    leak_pub = rcl_node->create_publisher<std_msgs::msg::Empty>(lolo_msgs::msg::Topics::LEAK_TOPIC, 1);

    //Status
    status_pub = rcl_node->create_publisher<lolo_msgs::msg::Status>(lolo_msgs::msg::Topics::EXTENDED_STATUS_TOPIC, 1);

    //Temperature
    temperature_pub = rcl_node->create_publisher<lolo_msgs::msg::Temperatures>(lolo_msgs::msg::Topics::EXTENDED_INTERNAL_TEMPERATURE_TOPIC, 1);

    //Pressure
    pressure_pub = rcl_node->create_publisher<lolo_msgs::msg::Pressures>(lolo_msgs::msg::Topics::EXTENDED_INTERNAL_PRESSURE_TOPIC, 1);
    
    //Control surfaces
    rudder_pub = rcl_node->create_publisher<lolo_msgs::msg::VolzServo>(lolo_msgs::msg::Topics::EXTENDED_RUDDER_FB_TOPIC, 1);
    elevator_pub = rcl_node->create_publisher<lolo_msgs::msg::VolzServo>(lolo_msgs::msg::Topics::EXTENDED_ELEVATOR_FB_TOPIC, 1);
    elevon_port_pub = rcl_node->create_publisher<lolo_msgs::msg::VolzServo>(lolo_msgs::msg::Topics::EXTENDED_PORT_ELEVON_FB_TOPIC, 1);
    elevon_strb_pub = rcl_node->create_publisher<lolo_msgs::msg::VolzServo>(lolo_msgs::msg::Topics::EXTENDED_STRB_ELEVON_FB_TOPIC, 1);

    //Thrusters
    thruster_port_pub = rcl_node->create_publisher<lolo_msgs::msg::VescFeedback>(lolo_msgs::msg::Topics::EXTENDED_PORT_THRUSTER_FB_TOPIC, 1);
    thruster_strb_pub = rcl_node->create_publisher<lolo_msgs::msg::VescFeedback>(lolo_msgs::msg::Topics::EXTENDED_STRB_THRUSTER_FB_TOPIC, 1);
    vertical_thruster_1_pub = rcl_node->create_publisher<lolo_msgs::msg::VescFeedback>(lolo_msgs::msg::Topics::EXTENDED_VERTICAL_THRUSTER_FRONT_PORT_FB_TOPIC, 1);
    vertical_thruster_2_pub = rcl_node->create_publisher<lolo_msgs::msg::VescFeedback>(lolo_msgs::msg::Topics::EXTENDED_VERTICAL_THRUSTER_FRONT_STRB_FB_TOPIC, 1);
    vertical_thruster_3_pub = rcl_node->create_publisher<lolo_msgs::msg::VescFeedback>(lolo_msgs::msg::Topics::EXTENDED_VERTICAL_THRUSTER_BACK_PORT_FB_TOPIC, 1);
    vertical_thruster_4_pub = rcl_node->create_publisher<lolo_msgs::msg::VescFeedback>(lolo_msgs::msg::Topics::EXTENDED_VERTICAL_THRUSTER_BACK_STRB_FB_TOPIC, 1);
    
    //Batteries
    battery1_pub = rcl_node->create_publisher<sensor_msgs::msg::BatteryState>(lolo_msgs::msg::Topics::EXTENDED_BATTERY_1_TOPIC, 1);
    battery2_pub = rcl_node->create_publisher<sensor_msgs::msg::BatteryState>(lolo_msgs::msg::Topics::EXTENDED_BATTERY_2_TOPIC, 1);

    //USBL
    usbl_pub = rcl_node->create_publisher<std_msgs::msg::Char>(lolo_msgs::msg::Topics::USBL_RECEIVED_CHR_TOPIC,20);

    //Satelite
    satelite_pub = rcl_node->create_publisher<std_msgs::msg::String>(lolo_msgs::msg::Topics::SATELITE_RECEIVED_TOPIC, 20);

    //Fuel cell
    fc_FCU_pub = rcl_node->create_publisher<lolo_msgs::msg::FuelCelldataFCU>(lolo_msgs::msg::Topics::FUELCELL_FCU_TOPIC, 20);
    fc_SYS_pub = rcl_node->create_publisher<lolo_msgs::msg::FuelCelldataSYS>(lolo_msgs::msg::Topics::FUELCELL_SYS_TOPIC, 20);


    printf("publishers created\n");
}

//parses data from lolo
void View::data_callback() {

    //Call parsing function based on message ID
    int msgID = lolo->messageID();
    switch (msgID) 
    {
        case CS_LEAK: {             lolo_callback_LEAK(); }; break; //Leak
        case CS_STATUS: {           lolo_callback_STATUS(); } break; //status
        case CS_RUDDER: {           lolo_callback_RUDDER(); } break; // rudder
        case CS_ELEVATOR: {         lolo_callback_ELEVATOR(); } break; //elevator
        case CS_ELEVON_PORT: {      lolo_callback_ELEVON_PORT(); } break; //Port elevon
        case CS_ELEVON_STRB: {      lolo_callback_ELEVON_STRB(); } break; //Strb elevon
        case CS_THRUSTER_PORT: {    lolo_callback_THRUSTER_PORT(); } break; //port thruster
        case CS_THRUSTER_STRB: {    lolo_callback_THRUSTER_STRB(); } break; //strb thruster
        case CS_VTHRUSTER_1:{       lolo_callback_VERTICAL_THRUSTER(msgID); } break; //vertical thruster
        case CS_VTHRUSTER_2:{       lolo_callback_VERTICAL_THRUSTER(msgID); } break; //vertical thruster
        case CS_VTHRUSTER_3:{       lolo_callback_VERTICAL_THRUSTER(msgID); } break; //vertical thruster
        case CS_VTHRUSTER_4:{       lolo_callback_VERTICAL_THRUSTER(msgID); } break; //vertical thruster
        case CS_BATTERY1: {         lolo_callback_BATTERY(msgID); } break; //battery1
        case CS_BATTERY2: {         lolo_callback_BATTERY(msgID); } break; //battery2
        case CS_MENUSTREAM: {       lolo_callback_MENUSTREAM(); } break; //Menu stream data
        case CS_USBL_RECEIVED: {    lolo_callback_USBL_RECEIVED(); } break; //USBL received}
        case CS_SATELITE_RECEIVED:  lolo_callback_SATELITE_RECEIVED(); break;
        case CS_TEMP: {             lolo_callback_TEMP();} break; //Data from temperature sensors inside lolo
        case CS_BARO: {             lolo_callback_BARO();} break; //Data from pressure sensors inside lolo
        case CS_EMSdFCUtoCapID: {   lolo_callback_FC_FCU();} break; //Data from fuel cell system (FCU)
        case CS_EMSdSYStoCapID: {   lolo_callback_FC_SYS();} break; //Data from fuel cell system (EMS & BMU)
    };
};

//parsing and publishing functions

void View::lolo_callback_LEAK() {
    std_msgs::msg::Empty msg;
    leak_pub->publish(msg);
}

void View::lolo_callback_STATUS() {
    
    lolo_msgs::msg::Status status_msg;

    status_msg.rc_signal     =    lolo->parse_byte();
    status_msg.voltage       =    lolo->parse_float();    //voltage 
    status_msg.current       =    lolo->parse_float();    //current
    status_msg.captain_leak  =    lolo->parse_byte();     //leak sensor
    status_msg.esc_leak      =    lolo->parse_byte();     //esc leak sensor
    status_msg.prevco_leak   =    lolo->parse_byte();     //Prevco leak sensor
    status_msg.edw_leak      =    lolo->parse_byte();     //edw leak sensor
    status_msg.battery1_leak =    lolo->parse_byte();     //battery1 leak sensor
    status_msg.battery2_leak =    lolo->parse_byte();     //battery2 leak sensor

    //ISB status
    status_msg.time_status                = lolo->parse_byte();
    status_msg.trigger_status             = lolo->parse_byte();
    status_msg.actuators_status           = lolo->parse_byte();
    status_msg.thrusters_status           = lolo->parse_byte();
    status_msg.vertical_thrusters_status  = lolo->parse_byte();
    status_msg.usbl_status                = lolo->parse_byte();
    status_msg.scientist_status           = lolo->parse_byte();
    status_msg.edw_status                 = lolo->parse_byte();
    status_msg.battery1_status            = lolo->parse_byte();
    status_msg.battery2_status            = lolo->parse_byte();

    //Aux outputs
    status_msg.aux_output     = lolo->parse_byte();
    status_msg.servo1_output  = lolo->parse_byte();
    status_msg.servo2_output  = lolo->parse_byte();
    status_msg.lumen_output   = lolo->parse_byte();
    status_msg.mbes_output    = lolo->parse_byte();

    //Emergency state
    switch (lolo->parse_byte())
    {
      case 0:
        status_msg.emergency_status = "NO EMERGENCY";
        break;
      case 1:
        status_msg.emergency_status = "LOW EMERGENCY";
        break;
      case 2:
        status_msg.emergency_status = "HIGH EMERGENCY";
        break;
    }

    //Control mode
    switch (lolo->parse_byte())
    {
      case 0:
        status_msg.control_mode = "STANDBY";
        break;
      case 1:
        status_msg.control_mode = "DISARMED";
        break;
      case 2:
        status_msg.control_mode = "ARMED";
        break;
    }

    //Control source
    switch (lolo->parse_byte())
    {
      case 0:
        status_msg.control_source = "NONE";
        break;
      case 1:
        status_msg.control_source = "MANUAL_CONTROL";
        break;
      case 2:
        status_msg.control_source = "ONBOARD_CONTROL";
        break;
      case 3:
        status_msg.control_source = "EXTERNAL_CONTROL";
        break;
      case 4:
        status_msg.control_source = "EMERGENCY_CONTROL";
        break;
    }

    //enabled / disabled thrusters
    status_msg.thrusters_enabled = lolo->parse_byte();
    status_msg.vertical_thrusters_enabled = lolo->parse_byte();

    //EDW
    status_msg.edw_armed = lolo->parse_byte();
    status_msg.edw_timer_armed = lolo->parse_byte();
    status_msg.edw_timer_time_left = lolo->parse_long();

    status_pub->publish(status_msg);
}

void View::lolo_callback_RUDDER() {
    //uint64_t timestamp    = lolo->parse_llong(); // timestamp from ISB
    //uint64_t sec = timestamp / 1000000;
    //uint64_t usec = timestamp % 1000000;
    lolo->parse_llong(); // Read timstamp and don't do anytihing with it

    float target = lolo->parse_float();               // actuator target in radians
    float angle = lolo->parse_float();                // actuator angle in radians
    float voltage = lolo->parse_float();              // voltage
    float current = lolo->parse_float();              // current
    //float humidity = lolo->parse_float();             // humidity (Does not exist)
    lolo->parse_float();             // humidity (Does not exist)
    float pcbtemp = lolo->parse_float();              // pcbtemp
    float motortemp = lolo->parse_float();            // motortemp

    //Detailed feedback
    lolo_msgs::msg::VolzServo feedback_msg;
    feedback_msg.target = target;
    feedback_msg.angle = angle;
    feedback_msg.voltage = voltage;
    feedback_msg.current = current;
    feedback_msg.pcbtemp = pcbtemp;
    feedback_msg.motortemp = motortemp;
    rudder_pub->publish(feedback_msg);
}

void View::lolo_callback_ELEVATOR() {
  //uint64_t timestamp    = lolo->parse_llong(); // timestamp from ISB
  //uint64_t sec = timestamp / 1000000;
  //uint64_t usec = timestamp % 1000000;
  lolo->parse_llong(); // Read timstamp and don't do anytihing with it
  float target = lolo->parse_float();               // actuator target in radians
  float angle = lolo->parse_float();                // actuator angle in radians
  float voltage = lolo->parse_float();              // voltage
  float current = lolo->parse_float();              // current
  //float humidity = lolo->parse_float();             // humidity
  lolo->parse_float();             // humidity (Does not exist)
  float pcbtemp = lolo->parse_float();              // pcbtemp
  float motortemp = lolo->parse_float();            // motortemp

  //Detailed feedback
  lolo_msgs::msg::VolzServo feedback_msg;
  feedback_msg.target = target;
  feedback_msg.angle = angle;
  feedback_msg.voltage = voltage;
  feedback_msg.current = current;
  //feedback_msg.humidity = humidity;
  feedback_msg.pcbtemp = pcbtemp;
  feedback_msg.motortemp = motortemp;
  elevator_pub->publish(feedback_msg); 
}

void View::lolo_callback_ELEVON_PORT() {
  //uint64_t timestamp    = lolo->parse_llong(); // timestamp from ISB
  //uint64_t sec = timestamp / 1000000;
  //uint64_t usec = timestamp % 1000000;
  lolo->parse_llong(); // Read timstamp and don't do anytihing with it
  float target = lolo->parse_float();               // actuator target in radians
  float angle = lolo->parse_float();                // actuator angle in radians
  float voltage = lolo->parse_float();              // voltage
  float current = lolo->parse_float();              // current
  //float humidity = lolo->parse_float();             // humidity
  lolo->parse_float();             // humidity (Does not exist)
  float pcbtemp = lolo->parse_float();              // pcbtemp
  float motortemp = lolo->parse_float();            // motortemp

  //Detailed feedback
  lolo_msgs::msg::VolzServo feedback_msg;
  feedback_msg.target = target;
  feedback_msg.angle = angle;
  feedback_msg.voltage = voltage;
  feedback_msg.current = current;
  //feedback_msg.humidity = humidity;
  feedback_msg.pcbtemp = pcbtemp;
  feedback_msg.motortemp = motortemp;
  elevon_port_pub->publish(feedback_msg);
}

void View::lolo_callback_ELEVON_STRB() {
  //uint64_t timestamp    = lolo->parse_llong(); // timestamp from ISB
  //uint64_t sec = timestamp / 1000000;
  //uint64_t usec = timestamp % 1000000;
  lolo->parse_llong(); // Read timstamp and don't do anytihing with it
  float target = lolo->parse_float();               // actuator target in radians
  float angle = lolo->parse_float();                // actuator angle in radians
  float voltage = lolo->parse_float();              // voltage
  float current = lolo->parse_float();              // current
  //float humidity = lolo->parse_float();             // humidity
  lolo->parse_float();             // humidity (Does not exist)
  float pcbtemp = lolo->parse_float();              // pcbtemp
  float motortemp = lolo->parse_float();            // motortemp

  //Detailed feedback
  lolo_msgs::msg::VolzServo feedback_msg;
  feedback_msg.target = target;
  feedback_msg.angle = angle;
  feedback_msg.voltage = voltage;
  feedback_msg.current = current;
  //feedback_msg.humidity = humidity;
  feedback_msg.pcbtemp = pcbtemp;
  feedback_msg.motortemp = motortemp;
  elevon_strb_pub->publish(feedback_msg);
}

void View::lolo_callback_THRUSTER_PORT() {
  //uint64_t timestamp    = lolo->parse_llong(); //timestamp from ISB
  //uint32_t sequence     = lolo->parse_long();  //sequence of this message
  lolo->parse_llong(); // Read timstamp and don't do anytihing with it
  lolo->parse_long();    
  float target          = lolo->parse_float();
  float rpm             = lolo->parse_float();
  float input_current   = lolo->parse_float();
  float input_voltage   = lolo->parse_float();
  float motor_current   = lolo->parse_float();
  float tempMosfet      = lolo->parse_float();

  //uint64_t sec = timestamp / 1000000;
  //uint64_t usec = timestamp % 1000000;

  lolo_msgs::msg::VescFeedback vesc_feedback_msg;
  vesc_feedback_msg.target_rpm = target;
  vesc_feedback_msg.rpm = rpm;
  vesc_feedback_msg.input_current = input_current;
  vesc_feedback_msg.input_voltage = input_voltage;
  vesc_feedback_msg.motor_current = motor_current;
  vesc_feedback_msg.temp_mosfet = tempMosfet;
  
  thruster_port_pub->publish(vesc_feedback_msg);
}

void View::lolo_callback_THRUSTER_STRB() {
  //uint64_t timestamp    = lolo->parse_llong(); //timestamp from ISB
  //uint32_t sequence     = lolo->parse_long();  //sequence of this message
  lolo->parse_llong(); // Read timstamp and don't do anytihing with it
  lolo->parse_long();      
  float target          = lolo->parse_float();
  float rpm             = lolo->parse_float();
  float input_current   = lolo->parse_float();
  float input_voltage   = lolo->parse_float();
  float motor_current   = lolo->parse_float();
  float tempMosfet      = lolo->parse_float();

  //uint64_t sec = timestamp / 1000000;
  //uint64_t usec = timestamp % 1000000;
  
  lolo_msgs::msg::VescFeedback vesc_feedback_msg;
  vesc_feedback_msg.target_rpm = target;
  vesc_feedback_msg.rpm = rpm;
  vesc_feedback_msg.input_current = input_current;
  vesc_feedback_msg.input_voltage = input_voltage;
  vesc_feedback_msg.motor_current = motor_current;
  vesc_feedback_msg.temp_mosfet = tempMosfet;
  
  thruster_strb_pub->publish(vesc_feedback_msg);
}

void View::lolo_callback_VERTICAL_THRUSTER(int thruster_id) {
  //uint64_t timestamp    = lolo->parse_llong(); //timestamp from ISB    
  lolo->parse_llong(); // Read timstamp and don't do anytihing with it
  float target          = lolo->parse_float();
  float rpm             = lolo->parse_float();
  float input_current   = lolo->parse_float();
  float input_voltage   = lolo->parse_float();
  float motor_current   = lolo->parse_float();
  float tempMosfet      = lolo->parse_float();
  
  lolo_msgs::msg::VescFeedback vesc_feedback_msg;
  vesc_feedback_msg.target_rpm = target;
  vesc_feedback_msg.rpm = rpm;
  vesc_feedback_msg.input_current = input_current;
  vesc_feedback_msg.input_voltage = input_voltage;
  vesc_feedback_msg.motor_current = motor_current;
  vesc_feedback_msg.temp_mosfet = tempMosfet;
  switch (thruster_id)
  {
    case CS_VTHRUSTER_1: vertical_thruster_1_pub->publish(vesc_feedback_msg); break;
    case CS_VTHRUSTER_2: vertical_thruster_2_pub->publish(vesc_feedback_msg); break;
    case CS_VTHRUSTER_3: vertical_thruster_3_pub->publish(vesc_feedback_msg); break;
    case CS_VTHRUSTER_4: vertical_thruster_4_pub->publish(vesc_feedback_msg); break;  
  default:
    break;
  }
}

void View::lolo_callback_BATTERY(int id) {
  // parse and publish battery information
  uint8_t battery_state = lolo->parse_byte(); //Battery state. Check ros message doc for definition
  float BatteryVoltage  = lolo->parse_float();
  float BatteryCurrent  = lolo->parse_float();
  float SOC             = lolo->parse_float();
  float CellVoltage_0   = lolo->parse_float();
  float CellVoltage_1   = lolo->parse_float();
  float CellVoltage_2   = lolo->parse_float();
  float CellVoltage_3   = lolo->parse_float();
  float CellVoltage_4   = lolo->parse_float();
  float CellVoltage_5   = lolo->parse_float();
  float CellVoltage_6   = lolo->parse_float();
  float CellVoltage_7   = lolo->parse_float();
  float CellVoltage_8   = lolo->parse_float();
  float CellVoltage_9   = lolo->parse_float();
  ///float temp1           = lolo->parse_float();
  ///float temp2           = lolo->parse_float();
  ///float temp3           = lolo->parse_float();
  ///float temp4           = lolo->parse_float();
  ///float temp5           = lolo->parse_float();

  sensor_msgs::msg::BatteryState battery_msg;  
  //std_msgs/Header header
  battery_msg.voltage = BatteryVoltage;
  battery_msg.current = BatteryCurrent;
  battery_msg.charge = SOC*210.0/100.0;
  battery_msg.capacity = 210;
  battery_msg.design_capacity = 210;
  battery_msg.percentage = SOC;
  battery_msg.power_supply_status = battery_state;
  battery_msg.power_supply_health = battery_msg.POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology = battery_msg.POWER_SUPPLY_TECHNOLOGY_LIFE;
  battery_msg.present = true;
  battery_msg.cell_voltage.push_back(CellVoltage_0);
  battery_msg.cell_voltage.push_back(CellVoltage_1);
  battery_msg.cell_voltage.push_back(CellVoltage_2);
  battery_msg.cell_voltage.push_back(CellVoltage_3);
  battery_msg.cell_voltage.push_back(CellVoltage_4);
  battery_msg.cell_voltage.push_back(CellVoltage_5);
  battery_msg.cell_voltage.push_back(CellVoltage_6);
  battery_msg.cell_voltage.push_back(CellVoltage_7);
  battery_msg.cell_voltage.push_back(CellVoltage_8);
  battery_msg.cell_voltage.push_back(CellVoltage_9);
  switch (id)
  {
    case CS_BATTERY1: 
      battery_msg.location = "Port battery";
      battery1_pub->publish(battery_msg); 
      break;
    case CS_BATTERY2: 
      battery_msg.location = "strb battery";
      battery2_pub->publish(battery_msg); 
      break;
  default:
    break;
  }
}

void View::lolo_callback_MENUSTREAM() {
  int length = lolo->parse_byte();
  std::string text = lolo->parse_string(length);
  printf("%s\n",text.c_str());
  std_msgs::msg::String msg;
  msg.data = text.c_str();
  menu_pub->publish(msg);
}

void View::lolo_callback_USBL_RECEIVED() {
    int length = lolo->parse_byte();
    std::cout << "USBL data received. length " << length << std::endl;
    for (int i=0;i<length;i++) {
        std_msgs::msg::Char msg;
        msg.data = lolo->parse_byte();
        usbl_pub->publish(msg);
    }
  }

void View::lolo_callback_SATELITE_RECEIVED() {
  int length = lolo->parse_byte();
  std::string text = lolo->parse_string(length);
  std::cout << "SATELITE data received. length " << length << " message: " << text << std::endl;
  std_msgs::msg::String msg;
  msg.data = text.c_str();
  satelite_pub->publish(msg);
}

void View::lolo_callback_TEMP() 
{
  lolo_msgs::msg::Temperatures temperature_msg;
  temperature_msg.captain_cpu = lolo->parse_byte();
  temperature_msg.time_cpu = lolo->parse_byte();
  temperature_msg.captain_top = lolo->parse_byte();
  temperature_msg.captain_eth = lolo->parse_byte();
  temperature_msg.captain_nuc = lolo->parse_byte();
  temperature_msg.usbl_isb = lolo->parse_byte();
  temperature_msg.actuator_cpu = lolo->parse_byte();
  temperature_msg.elevator_pcb = lolo->parse_byte();
  temperature_msg.elevator_motor = lolo->parse_byte();
  temperature_msg.rudder_motor = lolo->parse_byte();
  temperature_msg.rudder_pcb = lolo->parse_byte();
  temperature_msg.elevon_port_motor = lolo->parse_byte();
  temperature_msg.elevon_port_pcb = lolo->parse_byte();
  temperature_msg.elevon_strb_motor = lolo->parse_byte();
  temperature_msg.elevon_strb_pcb = lolo->parse_byte();
  temperature_msg.thruster_isb = lolo->parse_byte();
  temperature_msg.port_esc = lolo->parse_byte();
  temperature_msg.strb_esc = lolo->parse_byte();
  temperature_msg.vertical_thruster_isb = lolo->parse_byte();
  temperature_msg.vertical_thruster_1_esc = lolo->parse_byte();
  temperature_msg.vertical_thruster_2_esc = lolo->parse_byte();
  temperature_msg.vertical_thruster_3_esc = lolo->parse_byte();
  temperature_msg.vertical_thruster_4_esc = lolo->parse_byte();
  temperature_msg.prevco_isb = lolo->parse_byte();
  temperature_msg.edw_isb = lolo->parse_byte();
  temperature_msg.battery1_isb = lolo->parse_byte();
  temperature_msg.battery1_temp1 = lolo->parse_byte();
  temperature_msg.battery1_temp2 = lolo->parse_byte();
  temperature_msg.battery1_temp3 = lolo->parse_byte();
  temperature_msg.battery1_temp4 = lolo->parse_byte();
  temperature_msg.battery1_temp5 = lolo->parse_byte();
  temperature_msg.battery2_isb = lolo->parse_byte();
  temperature_msg.battery2_temp1 = lolo->parse_byte();
  temperature_msg.battery2_temp2 = lolo->parse_byte();
  temperature_msg.battery2_temp3 = lolo->parse_byte();
  temperature_msg.battery2_temp4 = lolo->parse_byte();
  temperature_msg.battery2_temp5 = lolo->parse_byte();
  temperature_pub->publish(temperature_msg);
}

void View::lolo_callback_BARO() {
  lolo_msgs::msg::Pressures pressure_msg;
  pressure_msg.usbl_isb = lolo->parse_float();
  pressure_msg.thrusters_isb = lolo->parse_float();
  pressure_msg.vertical_thrusters_isb  = lolo->parse_float();
  pressure_msg.prevco_isb = lolo->parse_float();
  pressure_msg.edw_isb = lolo->parse_float();
  pressure_msg.battery1_isb = lolo->parse_float();
  pressure_msg.battery2_isb = lolo->parse_float();
  pressure_pub->publish(pressure_msg);
}

void View::lolo_callback_FC_FCU() {
  //std::cout << "FC data receuved (FCU) " << std::endl;

  //CaptainRxData_t rxData;
  CaptainFCURxData_t rxDataFCU;

  // FCU packet
  unpackFieldsFromCom(lolo,
                      &rxDataFCU,
                      captainFCURxDataFields,
                      CAPTAIN_FCURXDATA_FIELD_COUNT);

  //Publish ROS message with data
  lolo_msgs::msg::FuelCelldataFCU fcu_msg;
  fcu_msg.anode_pressure = rxDataFCU.ANODE_PRESSURE;
  fcu_msg.cathode_pressure = rxDataFCU.CATHODE_PRESSURE;
  fcu_msg.coolant_valve_position = rxDataFCU.COOLANT_VALVE_POSITION;
  fcu_msg.o2_humidity = rxDataFCU.O2_HUMIDITY;
  fcu_msg.h2_humidity = rxDataFCU.H2_HUMIDITY;
  fcu_msg.h2_fresh_pressure = rxDataFCU.H2_FRESH_PRESSURE;
  fcu_msg.o2_fresh_pressure = rxDataFCU.O2_FRESH_PRESSURE;
  fcu_msg.anode_temperature = rxDataFCU.ANODE_TEMPERATURE;
  fcu_msg.cathode_temperature = rxDataFCU.CATHODE_TEMPERATURE;
  fcu_msg.o2_fresh_temp = rxDataFCU.O2_FRESH_TEMP;
  fcu_msg.h2_fresh_temp = rxDataFCU.H2_FRESH_TEMP;
  fcu_msg.o2_temp = rxDataFCU.O2_TEMP;
  fcu_msg.h2_temp = rxDataFCU.H2_TEMP;
  fcu_msg.stack_current = rxDataFCU.STACK_CURRENT;
  fcu_msg.stack_voltage = rxDataFCU.STACK_VOLTAGE;
  fcu_msg.bat_voltage = rxDataFCU.BAT_VOLTAGE;
  fcu_msg.min_cell_voltage = rxDataFCU.MIN_CELL_VOLTAGE;
  fcu_msg.max_cell_voltage = rxDataFCU.MAX_CELL_VOLTAGE;
  fcu_msg.avg_cell_voltage = rxDataFCU.AVG_CELL_VOLTAGE;
  fcu_msg.h2_concentration = rxDataFCU.H2_CONCENTRATION;
  fcu_msg.monitor_safety_state = rxDataFCU.MONITOR_SAFETY_STATE;
  fcu_msg.safety_fault = rxDataFCU.SAFETY_FAULT;
  fcu_msg.startup_status = rxDataFCU.STARTUP_STATUS;
  fcu_msg.shutdown_status = rxDataFCU.SHUTDOWN_STATUS;
  fcu_msg.feedback_control_setpoint = rxDataFCU.FEEDBACK_CONTROL_SETPOINT;
  fcu_msg.stack_current_setpoint = rxDataFCU.STACK_CURRENT_SETPOINT;
  fcu_msg.air_pump_knf_setpoint = rxDataFCU.AIR_PUMP_KNF_SETPOINT;
  fcu_msg.oxygen_cutoff_setpoint = rxDataFCU.OXYGEN_CUTOFF_SETPOINT;
  fcu_msg.h2_pump_knf_setpoint = rxDataFCU.H2_PUMP_KNF_SETPOINT;
  fcu_msg.coolant_pump_setpoint = rxDataFCU.COOLANT_PUMP_SETPOINT;
  fcu_msg.h2_fresh_setpoint = rxDataFCU.H2_FRESH_SETPOINT;
  fcu_msg.contactor_setpoint = rxDataFCU.CONTACTOR_SETPOINT;
  fcu_msg.purge_valve_setpoint = rxDataFCU.PURGE_VALVE_SETPOINT;
  fcu_msg.o2_fresh_setpoint = rxDataFCU.O2_FRESH_SETPOINT;
  fcu_msg.power_cycle_dcdc_cvm_setpoint = rxDataFCU.POWER_CYCLE_DCDC_CVM_SETPOINT;
  fcu_msg.safety_state = rxDataFCU.SAFETY_STATE;
  fcu_msg.safety_active_mask   = rxDataFCU.SAFETY_ACTIVE_MASK ;
  fcu_msg.safety_latched_mask = rxDataFCU.SAFETY_LATCHED_MASK;
  fcu_msg.safety_active_count = rxDataFCU.SAFETY_ACTIVE_COUNT;
  fcu_msg.safety_total_count = rxDataFCU.SAFETY_TOTAL_COUNT ;
  fcu_msg.ms_since_boot = rxDataFCU.MS_SINCE_BOOT;

  fc_FCU_pub->publish(fcu_msg);
  
};

void View::lolo_callback_FC_SYS() {
  //std::cout << "FC data receuved (EMS & BMU) " << std::endl;

  //CaptainRxData_t rxData;
  CaptainSystemRxData_t rxDataSys;

  // System (BMU+EMS) packet
  unpackFieldsFromCom(lolo,
                      &rxDataSys,
                      captainSystemRxDataFields,
                      CAPTAIN_SYSTEMRXDATA_FIELD_COUNT);
  
  lolo_msgs::msg::FuelCelldataSYS sys_msg;
  
  for(int i=0;i<8;i++) {
    sys_msg.bmu_cell_voltage[i] = rxDataSys.BMU_CELL_VOLTAGE[i];
  }
  sys_msg.bmu_battery_voltage = rxDataSys.BMU_BATTERY_VOLTAGE;
  sys_msg.bmu_battery_current = rxDataSys.BMU_BATTERY_CURRENT;
  sys_msg.bmu_soc = rxDataSys.BMU_SOC;
  sys_msg.bmu_status = rxDataSys.BMU_STATUS;
  sys_msg.bmu_temp_bms_board = rxDataSys.BMU_TEMP_BMS_BOARD;
  sys_msg.bmu_temp_sensor_1 = rxDataSys.BMU_TEMP_SENSOR_1;
  sys_msg.bmu_temp_sensor_2 = rxDataSys.BMU_TEMP_SENSOR_2;
  sys_msg.bmu_main_switch = rxDataSys.BMU_MAIN_SWITCH;
  sys_msg.bmu_allow_on = rxDataSys.BMU_ALLOW_ON;
  sys_msg.bmu_load_enable = rxDataSys.BMU_LOAD_ENABLE;
  sys_msg.bmu_charge_enable = rxDataSys.BMU_CHARGE_ENABLE;
  sys_msg.bmu_supply_voltage = rxDataSys.BMU_SUPPLY_VOLTAGE;
  sys_msg.bmu_current = rxDataSys.BMU_CURRENT;
  sys_msg.bmu_pressure = rxDataSys.BMU_PRESSURE;
  sys_msg.bmu_leak_detected = rxDataSys.BMU_LEAK_DETECTED;
  sys_msg.bmu_timestamp = rxDataSys.BMU_TIMESTAMP;
  sys_msg.ems_fcu_current = rxDataSys.EMS_FCU_current;
  sys_msg.ems_load_current = rxDataSys.EMS_Load_current;
  sys_msg.ems_system_state = rxDataSys.EMS_SYSTEM_STATE;
  sys_msg.ems_comms_status = rxDataSys.EMS_COMMS_STATUS;
  sys_msg.ems_power_setpoint = rxDataSys.EMS_POWER_SETPOINT;
  sys_msg.ems_h2_consumption = rxDataSys.EMS_H2_CONSUMPTION;
  sys_msg.ems_o2_consumption = rxDataSys.EMS_O2_CONSUMPTION;
  sys_msg.ems_stack_power = rxDataSys.EMS_STACK_POWER;
  sys_msg.ems_fcu_power = rxDataSys.EMS_FCU_POWER;
  sys_msg.ems_load_power = rxDataSys.EMS_LOAD_POWER;
  sys_msg.txkernelcount = rxDataSys.txKernelCount;

  fc_SYS_pub->publish(sys_msg);
};

