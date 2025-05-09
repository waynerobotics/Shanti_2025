//****************************************************************
// Author: Jose
// Date: October 2024
// File: joy2twist.cpp
// Description: This program subscribes to a joystick topic and publishes
//              to the turtle1/cmd_vel topic to control the turtlebot
//****************************************************************

#ifndef JOY2TWIST_HPP
#define JOY2TWIST_HPP

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>

class Joy2Twist : public rclcpp::Node
{
public:
  Joy2Twist();

private:
  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  
  // Parameters
  std::string twist_topic_;
  double linear_scale_;
  double angular_scale_;
  int linear_axis_;
  int angular_axis_;
  
  // Callback
  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
  
  // Parameter callback
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters);
  
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

#endif // JOY2TWIST_HPP