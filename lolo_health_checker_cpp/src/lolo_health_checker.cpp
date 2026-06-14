/**
 * @author Niklas Rolleberg
 * @author_email nrol@kth.se
 */
#include <chrono>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/int8.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include "smarc_msgs/msg/geofence_status_stamped.hpp"
#include "smarc_msgs/msg/topics.hpp"
#include "lolo_msgs/msg/topics.hpp"
#include "std_msgs/msg/float32.hpp"


using namespace std::chrono_literals;


class ErrorBucket {
private:
  float volume = 0; //0 = empty 1 = full
  float errorThreshold = 0.75;
  float drainRate = 0.05;
  float fillRate_bool = 0.24;
public:
  ErrorBucket(float _drainRate = 0.05, float _errorThreshhold = 0.75, float _fillRate_bool = 0.24) { //goes from full to empty in 20 updates
    drainRate = _drainRate;
    errorThreshold = _errorThreshhold;
    fillRate_bool = _fillRate_bool;
  };

  //Fill bucket with custom amout
  bool update(float fill = 0) {
    volume -= drainRate;
    if(volume < 0) volume = 0;
    volume += fill;
    if(volume > 1) volume = 1;
    return volume < errorThreshold;
  }

  //Fill bucket with standard amount
  bool update(bool OK = true) {
    volume -= drainRate;
    if(volume < 0) volume = 0;
    if(!OK) volume += fillRate_bool;
    if(volume > 1) volume = 1;
    return volume < errorThreshold;
  }

  bool getStatus() {
    return volume < errorThreshold;
  }

  float getVolume() {
    return volume;
  }

};

class HealthChecker : public rclcpp::Node {
 public:
  HealthChecker() : Node("evolo_health_checker_cpp") {

    //Subscribers
    // odom;
    _odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      smarc_msgs::msg::Topics::ODOM_TOPIC, 10, std::bind(&HealthChecker::odomCallback, this, std::placeholders::_1)
    );

    //Geofence sub
    _geofence_sub = this->create_subscription<smarc_msgs::msg::GeofenceStatusStamped>(
      smarc_msgs::msg::Topics::GEOFENCE_STATUS_TOPIC, 10, 
      std::bind(&HealthChecker::geofenceCallback, this, std::placeholders::_1));

    //depth
    _depth_sub = this->create_subscription<std_msgs::msg::Float32>(
      smarc_msgs::msg::Topics::DEPTH_TOPIC, 10,
      std::bind(&HealthChecker::depthCallback, this, std::placeholders::_1));

    //depth
    _altitude_sub = this->create_subscription<std_msgs::msg::Float32>(
      smarc_msgs::msg::Topics::ALTITUDE_TOPIC, 10,
      std::bind(&HealthChecker::altitudeCallback, this, std::placeholders::_1));

    // Timer for checking
    healthcheck_timer = this->create_wall_timer(
        1000ms, std::bind(&HealthChecker::timer_callback, this));

    //Health output pub
    _health_pub = this->create_publisher<std_msgs::msg::Int8>("/evolo/smarc/vehicle_health", 10);
  }

 public:
  
  //Settings?
  //TODO rosparam
  float dive_depth_threshold = -0.5; //(ENU)
  float altitude_limit = 0.5; //(ENU)
  float altitude_limit_at_speed = 1.5; //(ENU)
  double max_diveTime = 1800; //s
  float max_depth = -12; //m


 private:

  // Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odom_sub;
  rclcpp::Subscription<smarc_msgs::msg::GeofenceStatusStamped>::SharedPtr _geofence_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _depth_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _altitude_sub;
  
  rclcpp::TimerBase::SharedPtr healthcheck_timer;

  // Publishers
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _health_pub;


  // messages
  nav_msgs::msg::Odometry odom_message;
  bool odom_received = false; // True if we have received at least one odom message
  bool odom_updated = false; // True if we have received an upate since last check

  nav_msgs::msg::OccupancyGrid map_message;
  bool map_received = false;
  bool map_updated = false;

  smarc_msgs::msg::GeofenceStatusStamped geofence_message;
  bool geofence_received = false;
  bool geofence_updated = false;

  std_msgs::msg::Float32 altitude_message;
  bool altitude_received;
  bool altitude_updated;

  std_msgs::msg::Float32 depth_message;
  bool depth_received;
  bool depth_updated;
  float dive_start_time;

  //Error buckets
  ErrorBucket odom_value_error_bucket;
  ErrorBucket odom_time_error_bucket;
  ErrorBucket depth_value_error_bucket;
  ErrorBucket depth_time_error_bucket;
  ErrorBucket altitude_value_error_bucket;
  ErrorBucket altitude_time_error_bucket;
  ErrorBucket geofence_time_error_bucket;


  // -----------------------------------------------------------------------
  void odomCallback(const nav_msgs::msg::Odometry msg) {
    //std::cout << "Odom callback \n";
    odom_message = msg;
    odom_received = true;
    odom_updated = true;
  }

  // -----------------------------------------------------------------------
  void depthCallback(const std_msgs::msg::Float32 msg) {
    //std::cout << "Odom callback \n";
    depth_message = msg;
    depth_received = true;
    depth_updated = true;

    //Check if we are diving
    if (depth_message.data < 1.0) {
        dive_start_time = rclcpp::Clock{}.now().seconds();
    }
  }

  // -----------------------------------------------------------------------
  void altitudeCallback(const std_msgs::msg::Float32 msg) {
    //std::cout << "Odom callback \n";
    altitude_message = msg;
    altitude_received = true;
    altitude_updated = true;
  }

  // -----------------------------------------------------------------------
  void geofenceCallback(const smarc_msgs::msg::GeofenceStatusStamped msg) {
    //std::cout << "Geofence callback \n";
    geofence_message = msg;
    geofence_received = true;
    geofence_updated = true;
  }

  // -----------------------------------------------------------------------
  void timer_callback() {
    
    //Prevent error on startup
    if( !(odom_received && depth_received && altitude_received && geofence_received) )
    {
      std::cout << "NOT READY" << std::endl;
      std_msgs::msg::Int8 msg;
      msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_WAITING;
      _health_pub->publish(msg);
      return;
    }
    

    //Timeout checks
    bool timeout_error = false;
    if(!odom_time_error_bucket.update(odom_updated)) { timeout_error = true; std::cout << "TIMEOUT_ERROR: Odometry" << std::endl; }
    if(!geofence_time_error_bucket.update(geofence_updated))  { timeout_error = true; std::cout << "TIMEOUT_ERROR: Geofence" << std::endl; }
    if(!depth_time_error_bucket.update(depth_updated))  { timeout_error = true; std::cout << "TIMEOUT_ERROR: dpeth" << std::endl; }
    if(!altitude_time_error_bucket.update(altitude_updated))  { timeout_error = true; std::cout << "TIMEOUT_ERROR: altitude" << std::endl; }
        
    //Timeout error
    if(timeout_error) {
      //Timeout error
      std::cout << "Timout error" << std::endl;
      std_msgs::msg::Int8 msg;
      msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //ERROR
      _health_pub->publish(msg);
      return;
    }
    
    std_msgs::msg::Int8 health_msg;
    health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_READY; //All good

    //Data checks
    //Outside geofence
    if (geofence_message.status == geofence_message.STATUS_OUTSIDE) {
      health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //All bad
      std::cout << "Outsude Geofence" << std::endl;
    }
    else if (geofence_message.status == smarc_msgs::msg::GeofenceStatusStamped::STATUS_INACTIVE) {
      float vx = odom_message.twist.twist.linear.x;
      float vy = odom_message.twist.twist.linear.y;
      float speed = sqrt(vx*vx + vy*vy);

      //Speed
      if(speed > 0.5) {
        std::cout << "Too fast without active geofence" << std::endl;
        health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //All bad
      }

      //Depth
      if(odom_message.pose.pose.position.z < dive_depth_threshold) {
        std::cout << "Diving without geofence" << std::endl;
        health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //All bad
      }
    }

    //Max depth
    if(odom_message.pose.pose.position.z < max_depth) {
        std::cout << "Max depth exceded" << std::endl;
        health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //All bad
    }

    //dive time
    double now = rclcpp::Clock{}.now().seconds();
    if(now - dive_start_time > max_diveTime) {
      std::cout << "Max dive time exceded" << std::endl;
      health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //All bad
    }

    //Min altitude at speed
    float vx = odom_message.twist.twist.linear.x;
    float vy = odom_message.twist.twist.linear.y;
    float speed = sqrt(vx*vx + vy*vy);
    if(altitude_message.data < altitude_limit_at_speed && speed > 0.5) {
        std::cout << "minimum altitude at speed exceded" << std::endl;
        health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //All bad
    }
    
    //Min altitude 
    if(altitude_message.data < altitude_limit) {
        std::cout << "minimum altitude exceded" << std::endl;
        health_msg.data = smarc_msgs::msg::Topics::VEHICLE_HEALTH_ERROR; //All bad
    }
    
    _health_pub->publish(health_msg);

    //reset variables
    odom_updated = false;
    map_updated = false;
    geofence_updated = false;
    depth_updated = false;
    altitude_updated = false;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HealthChecker>());
  rclcpp::shutdown();

  return 0;
}
