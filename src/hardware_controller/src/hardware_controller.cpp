#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <string>
#include <vector>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <sys/ioctl.h>
#include <cmath>

struct WheelInterface
{
  float positions[2];
  float velocities[2];
};

using CallbackReturn = hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace hardware_controller
{

  class HardwareController : public hardware_interface::SystemInterface
  {
  public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override
    { 
      info_ = info;
      if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
        return CallbackReturn::ERROR;

      port_ = "/dev/ttyUSB0";
      baudrate_ = 115200;
      num_wheels_ = 2;

      positions_.resize(num_wheels_, 0.0);
      velocities_.resize(num_wheels_, 0.0);
      commands_.resize(num_wheels_, 0.0);
      directions_ = {1, 1}; 

      for (size_t i = 0; i < info_.joints.size(); ++i)
      {
        RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Joint[%ld] = %s", i, info_.joints[i].name.c_str());
      }

      return CallbackReturn::SUCCESS;
    }

    CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
    {
      RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Opening Serial Port: %s", port_.c_str());
      fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
      if (fd_ < 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Failed to open serial port: %s | %s", port_.c_str(), strerror(errno));
        return CallbackReturn::ERROR;
      }

      if (tcgetattr(fd_, &tty_) != 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Error %i from tcgetattr: %s", errno, strerror(errno));
        close(fd_);
        return CallbackReturn::ERROR;
      }

      tty_.c_cflag &= ~PARENB;        // Clear parity bit, disabling parity (most common)
      tty_.c_cflag &= ~CSTOPB;        // Clear stop field, only one stop bit used in communication (most common)
      tty_.c_cflag &= ~CSIZE;         // Clear all bits that set the data size
      tty_.c_cflag |= CS8;            // 8 bits per byte (most common)
      tty_.c_cflag &= ~CRTSCTS;       // Disable RTS/CTS hardware flow control (most common)
      tty_.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

      tty_.c_lflag &= ~ICANON;
      tty_.c_lflag &= ~ECHO;                                                        // Disable echo
      tty_.c_lflag &= ~ECHOE;                                                       // Disable erasure
      tty_.c_lflag &= ~ECHONL;                                                      // Disable new-line echo
      tty_.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
      tty_.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // Turn off s/w flow ctrl
      tty_.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes

      tty_.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
      tty_.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

      tty_.c_cc[VTIME] = 5; // Timeout in deciseconds (0.1s)
      tty_.c_cc[VMIN] = 0;  // Minimum number of characters to read

      cfsetospeed(&tty_, B115200);
      cfsetispeed(&tty_, B115200);

      tcflush(fd_, TCIFLUSH); // Flush input and output buffers

      if (tcsetattr(fd_, TCSANOW, &tty_) != 0)
      {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Error %i from tcsetattr: %s", errno, strerror(errno));
        close(fd_);
        return CallbackReturn::ERROR;
      }

      RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Serial Port Opened");
      return CallbackReturn::SUCCESS;
    }

    // CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
    // {
    //   if (fd_ >= 0)
    //   {
    //     close(fd_);
    //   }
    //   return CallbackReturn::SUCCESS;
    // }
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
    {
      RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Deactivating... waiting for 60 seconds before releasing interfaces");

      std::this_thread::sleep_for(std::chrono::seconds(60));  // Delay shutdown

      RCLCPP_INFO(rclcpp::get_logger("MyHardware"), "Continuing shutdown after delay");
      return hardware_interface::CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::StateInterface> export_state_interfaces() override
    {
      std::vector<hardware_interface::StateInterface> states;
      for (size_t i = 0; i < num_wheels_; ++i)
      {
        states.emplace_back(info_.joints[i].name, "position", &positions_[i]);
        states.emplace_back(info_.joints[i].name, "velocity", &velocities_[i]);
        std::cout << info_.joints[i].name << "\n";
      }
      for (const auto &state : states)
      {
        std::cout << "[STATE] Exported: " << state.get_name() << "____" << state.get_interface_name() << std::endl;
      }
      std::cout << "Joint names in info_.joints:\n";
      for (const auto& joint : info_.joints)
      {
        std::cout << joint.name << "\n";
      }
      return states;
    }

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override
    {
      std::vector<hardware_interface::CommandInterface> cmds;
      for (size_t i = 0; i < num_wheels_; ++i)
      {
        std::cout << info_.joints[i].name << "\n";
        cmds.emplace_back(info_.joints[i].name, "velocity", &commands_[i]);
      }
      

      for (const auto &cmd : cmds)
      {
        std::cout << "[COMMAND] Exported: " << cmd.get_name() << "____" << cmd.get_interface_name() << std::endl;
      }
      return cmds;
    }

    hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override
    {
      const uint8_t START_BYTE = 0xCC;
      const uint8_t END_BYTE = 0xDD;

      WheelInterface feedback;
      uint8_t buffer[sizeof(WheelInterface) + 2];
      if (::read(fd_, buffer, sizeof(buffer)) != sizeof(buffer))
        return return_type::ERROR;

      if (buffer[0] != 0xCC || buffer[sizeof(buffer) - 1] != 0xDD)
        return return_type::OK;

      size_t total_bytes_read = 0;
      ssize_t bytes_read = 0;  // ✅ Declare outside the loop

      while (total_bytes_read < sizeof(buffer)) {
        bytes_read = ::read(fd_, buffer + total_bytes_read, sizeof(buffer) - total_bytes_read);
        if (bytes_read < 0) {
          RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Error reading from serial port: %s", strerror(errno));
          return return_type::ERROR;
        }
        total_bytes_read += bytes_read;
      }

      tcflush(fd_, TCIFLUSH); // Flush after reading is complete


      if (bytes_read < 0)
        RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Error reading from serial port: %s", strerror(errno));
      return return_type::ERROR;

      total_bytes_read += bytes_read;

      if (buffer[0] == START_BYTE && buffer[sizeof(buffer) - 1] == END_BYTE)
      {
        std::memcpy(&feedback, buffer + 1, sizeof(WheelInterface));
        for (size_t i = 0; i < num_wheels_; ++i)
        {
          positions_[i] = feedback.positions[i] * M_PI / 180.0 * directions_[i];
          velocities_[i] = feedback.velocities[i] * M_PI / 180.0 * directions_[i];
        }

        RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Received positions: %f %f", positions_[0], positions_[1]);
        RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Received velocities: %f %f", velocities_[0], velocities_[1]);
      }
      else
      {
        RCLCPP_WARN(rclcpp::get_logger("HardwareController"), "Invalid feedback packet format");
      }
      RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Received positions: %f %f", positions_[0], positions_[1]);

      std::this_thread::sleep_for(std::chrono::milliseconds(20));

      return return_type::OK;
    }

    hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override
    {
      const uint8_t START_BYTE = 0xAA;
      const uint8_t END_BYTE = 0xBB;

      WheelInterface command;
      for (size_t i = 0; i < num_wheels_; ++i)
        command.velocities[i] = commands_[i] * (180.0 / M_PI) * directions_[i];

      uint8_t buffer[sizeof(WheelInterface) + 2];
      buffer[0] = START_BYTE;
      std::memcpy(buffer + 1, &command, sizeof(WheelInterface));
      buffer[sizeof(buffer) - 1] = END_BYTE;

      ssize_t written_bytes = ::write(fd_, buffer, sizeof(buffer));

      if (written_bytes != sizeof(buffer))
      {
        RCLCPP_ERROR(rclcpp::get_logger("HardwareController"), "Failed to send command");
        return return_type::ERROR;
      }
      RCLCPP_INFO(rclcpp::get_logger("HardwareController"), "Sent target velocities: %f %f",
                  command.velocities[0], command.velocities[1]);

      return return_type::OK;
    }

    
  private:
    std::string port_;
    struct termios tty_;
    int baudrate_;
    int fd_{-1};
    size_t num_wheels_{2};
    std::vector<int> directions_;
    std::vector<double> positions_, velocities_, commands_;
  };

}

PLUGINLIB_EXPORT_CLASS(hardware_controller::HardwareController, hardware_interface::SystemInterface)