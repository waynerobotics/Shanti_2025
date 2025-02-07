#include "joy2twist.hpp"

Joy2Twist::Joy2Twist() 
: Node("joy2twist")
{
  // Declare parameters with default values
  this->declare_parameter("twist_topic", "turtle1/cmd_vel");
  this->declare_parameter("linear_scale", 2.0);
  this->declare_parameter("angular_scale", 2.0);
  
  // Get parameter values
  twist_topic_ = this->get_parameter("twist_topic").as_string();
  linear_scale_ = this->get_parameter("linear_scale").as_double();
  angular_scale_ = this->get_parameter("angular_scale").as_double();
  
  // Create publisher with parameterized topic
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic_, 10);
  
  // Create subscription
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", 10, std::bind(&Joy2Twist::joyCallback, this, std::placeholders::_1));
    
  // Set up parameter callback
  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&Joy2Twist::parametersCallback, this, std::placeholders::_1));
}

void Joy2Twist::joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
{
  geometry_msgs::msg::Twist twist;
  
  // Use parameterized scales for motion
  twist.linear.x = linear_scale_ * joy_msg->axes[1];   // Left stick up/down
  twist.angular.z = angular_scale_ * joy_msg->axes[3];  // Right stick left/right

  twist_pub_->publish(twist);
}

rcl_interfaces::msg::SetParametersResult Joy2Twist::parametersCallback(
  const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  
  for (const auto &param : parameters) {
    if (param.get_name() == "twist_topic") {
      twist_topic_ = param.as_string();
      // Recreate publisher with new topic
      twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(twist_topic_, 10);
    } else if (param.get_name() == "linear_scale") {
      linear_scale_ = param.as_double();
    } else if (param.get_name() == "angular_scale") {
      angular_scale_ = param.as_double();
    }
  }
  
  return result;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joy2Twist>());
  rclcpp::shutdown();
  return 0;
}