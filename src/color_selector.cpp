#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ColorSelector : public rclcpp::Node
{
public:
  ColorSelector() : Node("color_selector")
  {
    color_publisher_ = this->create_publisher<std_msgs::msg::String>("/selected_color", 10);


    hand_pose_subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "/hand_pose", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received hand pose: %s", msg->data.c_str());
        publish_selected_color(msg->data);
      });
  }

          //guu,tyoki,paaaa,
          //target_blue,target_red,target_yellow
          
private:
  void publish_selected_color(const std::string &hand_pose)
  {
    auto msg = std::make_shared<std_msgs::msg::String>();
    if (hand_pose == "guu")
      msg->data = "target_blue";
    else if (hand_pose == "tyoki")
      msg->data = "target_red";
    else if (hand_pose == "paaaa")
      msg->data = "target_yellow";

    color_publisher_->publish(*msg);
    RCLCPP_INFO(this->get_logger(), "Published selected color: %s", msg->data.c_str());
    RCLCPP_INFO(this->get_logger(), "aaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\n");
  }


  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hand_pose_subscriber_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorSelector>());
  rclcpp::shutdown();
  return 0;
}

