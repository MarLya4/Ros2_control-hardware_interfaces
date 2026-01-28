#include "example_hardware_interface/example_system.hpp"

#include <termios.h>//configuracion del puerto serial
#include <fcntl.h> //open
#include <unistd.h>//close
#include <cstring> 
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>

#include <sstream>
#include <iomanip>
#include <cmath>


#include "pluginlib/class_list_macros.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"

#define LOG_IMPORTANTE(logger, fmt, ...) \
  RCLCPP_INFO(logger, "\033[1;35m" fmt "\033[0m",##__VA_ARGS__)

namespace example_hardware_interface{

hardware_interface::CallbackReturn ExampleSystem::on_init(
const hardware_interface::HardwareInfo & info){
  
  // Call base init and check
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(rclcpp::get_logger("ExampleSystem"), "SystemInterface::on_init failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  //Set numbers of joins declared in the URDF/hardware 
  size_t num_joints = info.joints.size();

  // Resize state/command vectors to match number of joints
  positions_.assign(num_joints, 0.0);
  velocities_.assign(num_joints, 0.0);
  total_ticks_.assign(num_joints, 0);
  prev_ticks_.assign(num_joints, 0.0);

  commands_.assign(num_joints, 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ExampleSystem::on_configure(
    const rclcpp_lifecycle::State &){

  // Read optional hardware parameters (overrides defaults)
  for (const auto & p : info_.hardware_parameters) {
    if (p.first == "serial_port") {
      port_ = p.second;
    } else if (p.first == "baudrate") {
      baudrate_ = std::stoi(p.second);
    } else if (p.first == "out_ticks_per_rev") {
      out_ticks_per_rev_ = std::stoi(p.second);
    } else if (p.first == "wheel_radius_m") {
      wheel_radius_m_ = std::stod(p.second);
    }else if (p.first == "ticks_per_sec") {
      ticks_per_sec_ = std::stod(p.second);
    }else if (p.first == "in_ticks_per_rev") {
      in_ticks_per_rev_ = std::stod(p.second);
    }else if (p.first == "gear_ratio") {
      gear_ratio_ = std::stod(p.second);
    }else if (p.first == "rpm_motor") {
      rpm_motor_ = std::stod(p.second);
    }
  }
    float in_max_rad_s_ = rpm_motor_ * 2.0 *M_PI / 60;
    out_max_rad_s_ = in_max_rad_s_/ gear_ratio_;
    out_ticks_per_rev_= in_ticks_per_rev_ * gear_ratio_;

  LOG_IMPORTANTE(rclcpp::get_logger("ExampleSystem"),
              "on_configure(): port=%s baud=%d out_ticks_per_rev=%d wheel_radius_m=%.4f out_max_rad_s=%.2frad/s gear_ratio_=%.2f",
              port_.c_str(), baudrate_, out_ticks_per_rev_, wheel_radius_m_, out_max_rad_s_,gear_ratio_ );

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ExampleSystem::on_cleanup(
  const rclcpp_lifecycle::State &){
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ExampleSystem::on_activate(
    const rclcpp_lifecycle::State &){
    RCLCPP_INFO(rclcpp::get_logger("ExampleSystem"), "Activando hardware...");

    if (!open_serial()) {
      RCLCPP_ERROR(
        rclcpp::get_logger("ExampleSystem"),
        "Fallo al abrir el puerto serial durante on_activate"
    );
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ExampleSystem::on_deactivate(
  const rclcpp_lifecycle::State &){ 
  RCLCPP_INFO(rclcpp::get_logger("ExampleSystem"), "Desactivando hardware...");
  close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ExampleSystem::export_state_interfaces(){
  std::vector<hardware_interface::StateInterface> interfaces;

  for (size_t i = 0; i < positions_.size(); i++)
  {
    interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &positions_[i]);

    interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocities_[i]);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
ExampleSystem::export_command_interfaces(){
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (size_t i = 0; i < commands_.size(); i++)
  {
    interfaces.emplace_back(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &commands_[i]);
  }

  return interfaces;
}


hardware_interface::return_type ExampleSystem::write(
    const rclcpp::Time &, const rclcpp::Duration &){

  if (serial_fd_ < 0) {
    RCLCPP_WARN(rclcpp::get_logger("ExampleSystem"), 
                "write(): Puerto serial no abierto");
    return hardware_interface::return_type::ERROR;
  }
  
  std::array<int, 2> motor_cmds;

  for(size_t i = 0; i<2; i++){

    float cmd = commands_[i];
    double w = std::clamp(cmd, -out_max_rad_s_, out_max_rad_s_);
    
    motor_cmds[i]= static_cast<int>(
      (w/out_max_rad_s_) * 400
    );
  }

  //formato
  std::ostringstream msg;
  msg << std::fixed << std::setprecision(3);
  msg << motor_cmds[0] << "," << motor_cmds[1] << "\n";

  std::string out = msg.str();

  // envio
  ssize_t bytes_sent = ::write(serial_fd_, out.c_str(), out.size());
  //if no se envio
  if (bytes_sent < 0) {
    RCLCPP_ERROR(
      rclcpp::get_logger("ExampleSystem"),
      "Error en write(): %s", strerror(errno)
    );
    return hardware_interface::return_type::ERROR;
  }
  //if se envio incompleto
  if (bytes_sent != (ssize_t)out.size()) {
    RCLCPP_WARN(
      rclcpp::get_logger("ExampleSystem"),
      "write(): Enviados %ld de %zu bytes",
      bytes_sent, out.size()
    );
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ExampleSystem::read(
  const rclcpp::Time & time , const rclcpp::Duration & period){

  if (!last_time_initialized_) {
    last_time_ = time;
    last_time_initialized_ = true;
    return hardware_interface::return_type::OK;
  }

  
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::ERROR;
  }
  poll_serial();


  double dt = (time - last_time_).seconds();
  if(dt <= 0){
    return hardware_interface::return_type::OK;
  }


  while (!rx_lines_.empty()) {
    std::string line = rx_lines_.front();
    rx_lines_.pop_front();

    int64_t left_ticks, right_ticks;

    if (sscanf(line.c_str(), "%ld,%ld", &left_ticks, &right_ticks) == 2) {
      //posicion
    double left_rad = ticks_to_rad(left_ticks);
    double right_rad = ticks_to_rad(right_ticks);

    positions_[0] = left_rad;
    positions_[2] = left_rad;
    positions_[1] = right_rad;
    positions_[3] = right_rad;

    //velocidad

    int64_t d_left  = left_ticks  - prev_ticks_[0];
    int64_t d_right = right_ticks - prev_ticks_[1];

    double d_left_rad  = ticks_to_rad(d_left);
    double d_right_rad = ticks_to_rad(d_right);


    double left_vel  = d_left_rad  / dt;
    double right_vel = d_right_rad / dt;

    velocities_[0] = left_vel;
    velocities_[2] = left_vel;
    velocities_[1] = right_vel;
    velocities_[3] = right_vel;

      //actualizacion de estado
    prev_ticks_[0] = left_ticks;
    prev_ticks_[1] = right_ticks;
    last_time_ = time;

    }
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::CallbackReturn ExampleSystem::on_shutdown(
    const rclcpp_lifecycle::State & /*previous_state*/){

  RCLCPP_INFO(
    rclcpp::get_logger("ExampleSystem"),
    "Closing serial connection.");

  close_serial();

  return hardware_interface::CallbackReturn::SUCCESS;
}

speed_t ExampleSystem::baudrate_to_constant(int baudrate){
  switch (baudrate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default:
      RCLCPP_WARN(
        rclcpp::get_logger("ExampleSystem"),
        "Baudrate no soportado: %d, usando 9600 por defecto", baudrate
      );
      return B9600;
  }
}

bool ExampleSystem::open_serial(){
  // Abrir el puerto serial
  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY );
  if (serial_fd_ < 0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("ExampleSystem"),
        "No se pudo abrir el puerto serial: %s (errno=%d)", port_.c_str(), errno);
    return false;
  }

  // Obtener configuracion actual
  struct termios tty;   

  if (tcgetattr(serial_fd_, &tty) != 0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("ExampleSystem"),
        "Error al obtener atributos del puerto serial.");
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  // Poner en modo RAW (sin procesamiento de caracteres)
  cfmakeraw(&tty);

  // Convertir baudrate
  speed_t speed = baudrate_to_constant(baudrate_);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  // Permitir lectura
  tty.c_cflag |= (CLOCAL | CREAD | O_NONBLOCK);
  tty.c_cflag &= ~PARENB;  // sin paridad
  tty.c_cflag &= ~CSTOPB;  // 1 bit de stop
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |=  CS8;     // 8 bits de datos
  tty.c_cflag &= ~HUPCL;   //evita que el puerto "cuelgue" el Arduino

  // Aplicar cambios
  if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
    RCLCPP_ERROR(
        rclcpp::get_logger("ExampleSystem"),
        "Error al aplicar configuración del puerto serial."
    );
    close(serial_fd_);
    serial_fd_ = -1;
    return false;
  }

  int status;
  ioctl(serial_fd_, TIOCMGET, &status);
  status |= TIOCM_DTR;
  status |= TIOCM_RTS;
  ioctl(serial_fd_, TIOCMSET, &status);

  // Vaciar buffer
  tcflush(serial_fd_, TCIOFLUSH);

  RCLCPP_INFO(
      rclcpp::get_logger("ExampleSystem"),
      "Puerto serial %s abierto correctamente.", port_.c_str()
  );

  return true;
}

void ExampleSystem::close_serial(){
  if(serial_fd_ >= 0){
    close(serial_fd_);
    serial_fd_ = -1;

    RCLCPP_INFO(
      rclcpp::get_logger("ExampleSystem"),
      "Puerto serial cerrado"
    );
  }
}

void ExampleSystem::poll_serial(){
  char buf[128];
  ssize_t n = ::read(serial_fd_, buf, sizeof(buf));

  if (n <= 0) return;

  for (ssize_t i = 0; i < n; ++i) {
    char c = buf[i];

    if (c == '\n') {
      if (!rx_buffer_.empty() && rx_buffer_.back() == '\r') {
        rx_buffer_.pop_back();
      }
      rx_lines_.push_back(rx_buffer_);
      rx_buffer_.clear();
    } else {
      rx_buffer_ += c;
      if (rx_buffer_.size() > 256) rx_buffer_.clear();
    }
  }
}

double ExampleSystem::ticks_to_rad(int64_t ticks) const{
  return 
    (static_cast<double>(ticks / in_ticks_per_rev_))*(2* M_PI) / gear_ratio_;
}

}  // namespace example_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(example_hardware_interface::ExampleSystem, hardware_interface::SystemInterface)
