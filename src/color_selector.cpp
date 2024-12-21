// Copyright 2024 cit_sazae24
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


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
  //色選択
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

