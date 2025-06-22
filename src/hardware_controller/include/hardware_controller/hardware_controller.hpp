#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vector>
#include <string>

class HardwareController : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  std::string port_;
  int baudrate_;
  int fd_ = -1;

  // For two wheels: left and right
  enum WheelIndex { LEFT = 0, RIGHT = 1 };

  std::vector<double> positions_{0.0, 0.0};     // wheel positions in radians
  std::vector<double> velocities_{0.0, 0.0};    // wheel velocities in rad/s
  std::vector<double> commands_{0.0, 0.0};      // target wheel velocities (rad/s)


};
