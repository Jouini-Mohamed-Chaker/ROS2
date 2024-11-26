#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class LineFollowerNode : public rclcpp::Node
{
public:
  LineFollowerNode() : Node("line_follower_node")
  {
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/image_raw", 10, std::bind(&LineFollowerNode::image_callback, this, std::placeholders::_1));
  }

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received image");
    cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
    cv::Mat gray, binary;
    cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray, binary, 100, 255, cv::THRESH_BINARY_INV);

    cv::Moments m = cv::moments(binary, true);
    double cx = m.m10 / m.m00;
    double cy = m.m01 / m.m00;

    RCLCPP_INFO(this->get_logger(), "Centroid: (%f, %f)", cx, cy);

    auto twist_msg = geometry_msgs::msg::Twist();
    if (m.m00 > 0)
    {
      double error = cx - (image.cols / 2);
      twist_msg.linear.x = 0.2;
      twist_msg.angular.z = -error / 100;
      RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%f, angular.z=%f", twist_msg.linear.x, twist_msg.angular.z);
    }
    else
    {
      twist_msg.linear.x = 0.2;
      twist_msg.angular.z = 0.0;
      RCLCPP_INFO(this->get_logger(), "Publishing cmd_vel: linear.x=%f, angular.z=%f", twist_msg.linear.x, twist_msg.angular.z);
    }

    pub_->publish(twist_msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LineFollowerNode>());
  rclcpp::shutdown();
  return 0;
}