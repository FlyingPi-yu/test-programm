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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to regist er a
 * member function as a callback from the timer. */

class Eingangssignal : public rclcpp::Node
{

public:

  Eingangssignal()
  : Node("IMU_Signal"), count_(0)
  {
    time_start = rclcpp::Clock().now().nanoseconds();
    publisher_ = this->create_publisher<std_msgs::msg::UInt64>("x_Ist", 10);
    timer_ = this->create_wall_timer(
      5ms, std::bind(&Eingangssignal::IMU_200Hz, this));
  }



private:
  u_int64_t time_start;

  void IMU_200Hz()
  {
    auto message = std_msgs::msg::UInt64();
// Uint64
    rclcpp::Time time_now = rclcpp::Clock().now();
    message.data = time_now.nanoseconds();
//  RCLCPP_INFO(this->/*get_logger(), */"'%s' bekommen", message.data.c_str());
    publisher_->publish(message);
  }


  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr publisher_;

  size_t count_;
};

// Memory allocator hier anwenden
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  //eine single-Task Executor erstellen und spin die Node.
  rclcpp::spin(std::make_shared<Eingangssignal>());
  rclcpp::shutdown();
  return 0;
}
