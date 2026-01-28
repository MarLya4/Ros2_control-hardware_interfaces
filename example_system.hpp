#pragma once

#include <vector>
#include <string>
#include <memory>
#include <mutex>
#include <deque>
#include <termios.h> //configuracion del puerto serial

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"

namespace example_hardware_interface
{

class ExampleSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ExampleSystem);
  
    // Lifecycle callbacks
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

    //Interfaces export
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  
  // Main I/O
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  
// Serial helpers
  bool open_serial();
  void close_serial();
  bool write_serial(const std::string & data);
  void poll_serial();

  // Parameters / constants
  std::string port_ = "/dev/ttyUSB0";
  int baudrate_ = 9600;
   // Serial fd
  int serial_fd_ = -1;
  speed_t baudrate_to_constant(int baudrate);
  
  std::string rx_buffer_;
  std::deque<std::string> rx_lines_;
  
  int rpm_motor_ = 76;
  float gear_ratio_ = 1.5;
  int out_ticks_per_rev_ = 3150;  //medicion despues del tren de engranes.
  double wheel_radius_m_ = 0.034; 
  double ticks_to_rad_ ;

  double  ticks_per_sec_ = 2880; //medicion de ticks/s de motores. 
  int in_ticks_per_rev_= 2100; //medicion de ticks dalida de motor antes de  tren de engranes
  float out_max_rad_s_ = 0.0;

  double ticks_to_rad(int64_t ticks) const;
  // Buffers / variables for interfaces
  std::vector<double> positions_;   // radians
  std::vector<double> velocities_;   // rad/s

  std::vector<double> commands_;   // rad/s (velocity commands)

  std::vector<int64_t> total_ticks_;
  std::vector<double> prev_ticks_;

    // Timing
  rclcpp::Time last_time_;
  bool last_time_initialized_ = false;
};

}  // namespace example_hardware_interface

