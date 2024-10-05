#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <memory>

class Joy2Turtle : public rclcpp::Node
{
public:
  Joy2Turtle() : Node("joy2turtle"), l_scale_(2.0), a_scale_(2.0)
  {
    // To Do : Add parameters for topic turtle1 and linear and angular scale
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&Joy2Turtle::joyCallback, this, std::placeholders::_1));

  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  double l_scale_, a_scale_;

  void joyCallback(const sensor_msgs::msg::Joy::SharedPtr joy_msg)
  {
    geometry_msgs::msg::Twist twist;
    
    // Assuming left stick for linear motion (forward/backward)
    // and right stick for angular motion (left/right)
    // Adjust indices if your joystick mapping is different
    twist.linear.x = l_scale_ * joy_msg->axes[1];  // Left stick up/down
    twist.angular.z = a_scale_ * joy_msg->axes[3]; // Right stick left/right

    twist_pub_->publish(twist);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Joy2Turtle>());
  rclcpp::shutdown();
  return 0;
}