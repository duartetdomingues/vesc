// Copyright 2020 F1TENTH Foundation
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//
//   * Neither the name of the {copyright_holder} nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// -*- mode:c++; fill-column: 100; -*-

#include "vesc_ackermann/ackermann_to_vesc.hpp"

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <std_msgs/msg/float64.hpp>
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <sstream>
#include <string>

namespace vesc_ackermann
{

  using ackermann_msgs::msg::AckermannDriveStamped;
  using std::placeholders::_1;
  using std_msgs::msg::Float64;

  AckermannToVesc::AckermannToVesc(const rclcpp::NodeOptions &options)
      : Node("ackermann_to_vesc_node", options)
  {
    // declare parameters
    declare_parameter("speed_to_erpm_gain", 0.0);
    declare_parameter("speed_to_erpm_offset", 0.0);
    declare_parameter("steering_angle_to_servo_gain", 0.0);
    declare_parameter("steering_angle_to_servo_offset", 0.0);

    // get conversion parameters
    speed_to_erpm_gain_ = get_parameter("speed_to_erpm_gain").get_value<double>();
    speed_to_erpm_offset_ = get_parameter("speed_to_erpm_offset").get_value<double>();
    steering_to_servo_gain_ = get_parameter("steering_angle_to_servo_gain").get_value<double>();
    steering_to_servo_offset_ = get_parameter("steering_angle_to_servo_offset").get_value<double>();

    RCLCPP_INFO(
        this->get_logger(),
        "Parameters: speed_to_erpm_gain = %f, speed_to_erpm_offset = %f, "
        "steering_angle_to_servo_gain = %f, steering_angle_to_servo_offset = %f",
        speed_to_erpm_gain_,
        speed_to_erpm_offset_,
        steering_to_servo_gain_,
        steering_to_servo_offset_);

    // get parameter from /joy_teleop node

    parameters_client_joy = std::make_shared<rclcpp::SyncParametersClient>(this, "joy_teleop");

   
    if (!parameters_client_joy->service_is_ready())
    {
      RCLCPP_WARN(this->get_logger(), "Joy teleop service is not ready. Waiting...");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Joy teleop service is ready.");
    }

    if (!parameters_client_joy->wait_for_service(std::chrono::seconds(5)))
    {
      RCLCPP_WARN(this->get_logger(), "Joy teleop not found!");
      joy_active = false;
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Joy teleop found!");
      joy_active = true;
    }

    // create publishers to vesc electric-RPM (speed) and servo commands
    erpm_pub_ = create_publisher<Float64>("commands/motor/speed", 10);
    duty_pub_ = create_publisher<Float64>("commands/motor/duty_cycle", 10);
    servo_pub_ = create_publisher<Float64>("commands/servo/position", 10);

    // subscribe to ackermann topic
    ackermann_sub_ = create_subscription<AckermannDriveStamped>(
        "ackermann_cmd", 10, std::bind(&AckermannToVesc::ackermannCmdCallback, this, _1));
  }

  void AckermannToVesc::ackermannCmdCallback(const AckermannDriveStamped::SharedPtr cmd)
  {

    if (cmd->drive.acceleration < 0.0 )
    {
      cmd->drive.acceleration = 0.0;
    }
    if (cmd->drive.jerk < 0.0 )
    {
      cmd->drive.jerk = 0.0;
    }


    // calc vesc electric RPM (speed)
    Float64 erpm_msg;
    // erpm_msg.data = speed_to_erpm_gain_ * (cmd->drive.speed) + speed_to_erpm_offset_;
    erpm_msg.data = speed_to_erpm_gain_ * (cmd->drive.acceleration - cmd->drive.jerk);

    Float64 duty_msg;
    duty_msg.data = (cmd->drive.acceleration - cmd->drive.jerk);

    Float64 servo_msg;
    servo_msg.data = steering_to_servo_offset_ + cmd->drive.steering_angle;

    // publish
    if (rclcpp::ok())
    {
      // erpm_pub_->publish(erpm_msg);
      servo_pub_->publish(servo_msg);
      duty_pub_->publish(duty_msg);
    }
  }

} // namespace vesc_ackermann

#include "rclcpp_components/register_node_macro.hpp" // NOLINT

RCLCPP_COMPONENTS_REGISTER_NODE(vesc_ackermann::AckermannToVesc)
