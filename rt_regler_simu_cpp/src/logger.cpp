// Copyright 2020 DMT Technische Universität Dresden
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

#include <cinttypes>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/u_int64.hpp"


// komplett NICHT echtzeit-sichere Node für Logging und für Speicher der Latenz Ergebnisse
int main(int argc, char * argv[])
{
      setbuf(stdout, NULL);
      rclcpp::init(argc, argv);
      auto Latenz_Logger_Knote = rclcpp::Node::make_shared("Latenzen_logger");
      std::string filename = "IMU_Regler_logger_rt_results.csv";

      std::ofstream fstream;
      {
        fstream.open("IMU_Regler_logger_rt_results.csv", std::ios_base::out);
        fstream << "iteration, timestamp, latency" << std::endl;
        fstream.close();
      }


       size_t i = 0;

       // In Callback Funktion wird die Differenz der Zeitstampek von Abschicken
       // der x_Ist und der x_Soll in .CSV Datei geschrieben.
       auto logger_callback =
         [&i](const std_msgs::msg::UInt64::SharedPtr msg) {
           printf("Momentane Latenz: %" PRIu64 " ns\n", msg->data);

           std::ofstream fstream;
           fstream.open("IMU_Regler_logger_rt_results.csv", std::ios_base::app);
           fstream << i << "," << rclcpp::Clock().now().nanoseconds() << "," << msg->data << " " << std::endl;
           fstream.close();
           ++i;
         };

        auto Produkoller = Latenz_Logger_Knote->create_subscription<std_msgs::msg::UInt64>(
            "x_Soll", 100, logger_callback);

        printf("Logger Knote ist bereit.\n");
        rclcpp::spin(Latenz_Logger_Knote);


        rclcpp::shutdown();
}
