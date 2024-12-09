#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class ColorSelector : public rclcpp::Node
{
public:
  ColorSelector() : Node("color_selector")
  {
    auto msg = std::make_shared<std_msgs::msg::String>();
    
    msg->data = "target_black";  // 指定する色
  	//target_blue, target_red, target_yellow;

    color_publisher_ = this->create_publisher<std_msgs::msg::String>("/selected_color", 10);

    // 一定間隔でメッセージを送信するタイマーを設定
    timer_ = this->create_wall_timer(
      std::chrono::seconds(10),
      [this, msg]() {
        color_publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Color message published: %s", msg->data.c_str());
        RCLCPP_INFO(this->get_logger(), "aaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\naaaaaaaaaaaaaaaaa\n");
        timer_->cancel();	//一回だけ実行
      });
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr color_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ColorSelector>());
  rclcpp::shutdown();
  return 0;
}

