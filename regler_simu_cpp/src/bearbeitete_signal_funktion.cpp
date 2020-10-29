// Copyright 2016 Open Source Robotics Foundation, Inc.
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

#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class Ausgangssignal : public rclcpp::Node
{
public:
  Ausgangssignal()
  : Node("Regler_Signal"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("x_Soll", 5);
    subscription_ = this->create_subscription<std_msgs::msg::UInt64>(
      "x_Ist", 5, std::bind(&Ausgangssignal::x_Ist_bearbeiten, this, _1));
  }

private:
  size_t count_;
  uint64_t time_start;
  void x_Ist_bearbeiten(const std_msgs::msg::UInt64::SharedPtr msg)
  {
    int i = 1;
	do{
		i++;
        if( i == 99) {
        //Nach Simulationsrechnung schicke die Node eine Ausgangssignal für weitere Noden.
    //    RCLCPP_INFO(this->get_logger(), "'%i' ist ankommen",msg->data;
        auto message = std_msgs::msg::UInt64();
        time_start = msg->data;
        rclcpp::Time time_now = rclcpp::Clock().now();
        message.data = time_now.nanoseconds() - time_start;
        // printf(msg->data.c_str());
        publisher_->publish(message);
        break;
        }
	}while(i<=100);
  }

  //subscrib die Eingangssignal
  rclcpp::Subscription<std_msgs::msg::UInt64>::SharedPtr subscription_;
  //bearbeitete Information weitergeben
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ausgangssignal>());
  rclcpp::shutdown();
  return 0;
}
