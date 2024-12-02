#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ColorSelector : public rclcpp::Node
{
public:
  ColorSelector() : Node("color_selector")
  {

    auto msg = std::make_shared<std_msgs::msg::String>();
    msg->data = "target_blue";  // 指定する色

    // 色を`color_detection`ノードに送信
    color_publisher_ = this->create_publisher<std_msgs::msg::String>("/selected_color", 10);
    color_publisher_->publish(*msg);
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorSelector>());
  rclcpp::shutdown();
  return 0;
}

