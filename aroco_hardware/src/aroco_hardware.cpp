// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license


// std
#include <memory>
#include <thread>
#include <string>
#include <fstream>

// romea
#include "romea_mobile_base_hardware/hardware_info.hpp"

// ros
#include "rclcpp/rclcpp.hpp"


// local
#include "aroco_hardware/aroco_hardware.hpp"

namespace
{
const double WHEEL_LINEAR_SPEED_EPSILON = 0.01;
const double WHEEL_STEERING_ANGLE_EPSILON = 0.03;

const uint32_t FRONT_WHEEL_LINEAR_SPEEDS_COMMAND_ID = 0x15;
const uint32_t REAR_WHEEL_LINEAR_SPEEDS_COMMAND_ID = 0x16;
const uint32_t FRONT_STEERING_ANGLE_COMMAND_ID = 0x17;
const uint32_t REAR_STEERING_ANGLE_COMMAND_ID = 0x18;

const uint32_t FRONT_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID = 0x25;
const uint32_t REAR_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID = 0x1F;
const uint32_t STEERING_ANGLES_MEASUREMENT_ID = 0x26;

const uint32_t START_STOP_ID = 0x10;

const std::chrono::milliseconds TIMEOUT(5);

}  // namespace

namespace romea
{

//-----------------------------------------------------------------------------
ArocoHardware::ArocoHardware()
: HardwareSystemInterface2AS4WD(),
  can_receiver_thread_(nullptr),
  can_receiver_thread_run_(false),
  can_sender_("can0"),
  can_receiver_("can0"),
  sended_frame_data_(),
  received_frame_data_(),
  front_wheel_radius_(0),
  rear_wheel_radius_(0),
  front_axle_steering_angle_measure_(0),
  rear_axle_steering_angle_measure_(0),
  front_left_wheel_linear_speed_measure_(0),
  front_right_wheel_linear_speed_measure_(0),
  rear_left_wheel_linear_speed_measure_(0),
  rear_right_wheel_linear_speed_measure_(0),
  front_axle_steering_angle_command_(0),
  rear_axle_steering_angle_command_(0),
  front_left_wheel_linear_speed_command_(0),
  front_right_wheel_linear_speed_command_(0),
  rear_left_wheel_linear_speed_command_(0),
  rear_right_wheel_linear_speed_command_(0)
{
#ifndef NDEBUG
  open_log_file_();
  write_log_header_();
#endif
}

//-----------------------------------------------------------------------------
hardware_interface::return_type ArocoHardware::connect_()
{
  RCLCPP_INFO(rclcpp::get_logger("ArocoHardware"), "Init communication with robot");

  send_null_command_();
  start_can_receiver_thread_();
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type ArocoHardware::disconnect_()
{
  RCLCPP_INFO(rclcpp::get_logger("ArocoHardware"), "Close communication with robot");

  send_null_command_();
  stop_can_receiver_thread_();
  return hardware_interface::return_type::OK;
}
//-----------------------------------------------------------------------------
hardware_interface::return_type ArocoHardware::load_info_(
  const hardware_interface::HardwareInfo & hardware_info)
{
  try {
    front_wheel_radius_ = get_parameter<double>(hardware_info, "front_wheel_radius");
    rear_wheel_radius_ = get_parameter<double>(hardware_info, "rear_wheel_radius");
    return hardware_interface::return_type::OK;
  } catch (std::runtime_error & e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("ArocoHardware"), e.what());
    return hardware_interface::return_type::ERROR;
  }
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type ArocoHardware::read()
#else
hardware_interface::return_type ArocoHardware::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  RCLCPP_INFO(rclcpp::get_logger("ArocoHardware"), "Read data from robot");

  set_hardware_state_();

  std::cout << "wheels speeds " <<
    front_left_wheel_linear_speed_measure_.load() << " " <<
    front_right_wheel_linear_speed_measure_.load() << " " <<
    rear_left_wheel_linear_speed_measure_.load() << " " <<
    rear_right_wheel_linear_speed_measure_.load() << std::endl;

  std::cout << "steering angless " <<
    front_axle_steering_angle_measure_.load() << " " <<
    rear_axle_steering_angle_measure_.load() << std::endl;


  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
#if ROS_DISTRO == ROS_GALACTIC
hardware_interface::return_type ArocoHardware::write()
# else
hardware_interface::return_type ArocoHardware<HardwareInteface>::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
#endif
{
  RCLCPP_INFO(rclcpp::get_logger("ArocoHardware"), "Send command to robot");
  return hardware_interface::return_type::OK;

  get_hardware_command_();

  if (is_drive_enable_()) {
    std::cout << " send command " << std::endl;
    send_command_();

    std::cout << "wheels speeds command " <<
      front_left_wheel_linear_speed_command_ << " " <<
      front_right_wheel_linear_speed_command_ << " " <<
      rear_left_wheel_linear_speed_command_ << " " <<
      rear_right_wheel_linear_speed_command_ << std::endl;

    std::cout << "wheels angless command " <<
      front_axle_steering_angle_command_ << " " <<
      rear_axle_steering_angle_command_ << std::endl;
  }

#ifndef NDEBUG
  write_log_data_();
#endif

  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
void ArocoHardware::get_hardware_command_()
{
  HardwareCommand2AS4WD command = hardware_interface_->get_command();

  front_axle_steering_angle_command_ = command.frontAxleSteeringAngle;
  rear_axle_steering_angle_command_ = command.frontAxleSteeringAngle;

  front_left_wheel_linear_speed_command_ =
    command.frontLeftWheelSpinningSetPoint * front_wheel_radius_;
  front_right_wheel_linear_speed_command_ =
    command.frontRightWheelSpinningSetPoint * front_wheel_radius_;
  rear_left_wheel_linear_speed_command_ =
    command.rearLeftWheelSpinningSetPoint * rear_wheel_radius_;
  rear_right_wheel_linear_speed_command_ =
    command.rearRightWheelSpinningSetPoint * rear_wheel_radius_;
}

//-----------------------------------------------------------------------------
void ArocoHardware::set_hardware_state_()
{
  HardwareState2AS4WD state;

  state.frontAxleSteeringAngle = front_axle_steering_angle_measure_;
  state.rearAxleSteeringAngle = rear_axle_steering_angle_measure_;

  state.frontLeftWheelSpinningMotion.velocity =
    front_left_wheel_linear_speed_measure_ / front_wheel_radius_;
  state.frontRightWheelSpinningMotion.velocity =
    front_right_wheel_linear_speed_measure_ / front_wheel_radius_;
  state.rearLeftWheelSpinningMotion.velocity =
    rear_left_wheel_linear_speed_measure_ / rear_wheel_radius_;
  state.frontRightWheelSpinningMotion.velocity =
    rear_right_wheel_linear_speed_measure_ / rear_wheel_radius_;

  hardware_interface_->set_state(state);
}


//-----------------------------------------------------------------------------
void ArocoHardware::receive_data_()
{
  while (rclcpp::ok() && can_receiver_thread_run_) {
    try {
      drivers::socketcan::CanId receive_id = can_receiver_.
        receive(received_frame_data_.data(), TIMEOUT);

      switch (receive_id.identifier()) {
        case FRONT_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID:
          decode_front_wheel_speeds_();
          break;
        case REAR_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID:
          decode_rear_wheel_speeds_();
          break;
        case STEERING_ANGLES_MEASUREMENT_ID:
          decode_steering_angles_();
          break;
      }
    } catch (const std::exception & ex) {
      RCLCPP_WARN_STREAM(rclcpp::get_logger("ArocoHardware"), "Error receiving CAN message");
    }
  }
}

//-----------------------------------------------------------------------------
bool ArocoHardware::send_command_()
{
  return send_start_() &&   // why send start at each command?
         send_front_wheel_speeds_() &&
         send_rear_wheel_speeds_() &&
         send_front_steering_angle_() &&
         send_rear_steering_angle_();
}

//-----------------------------------------------------------------------------
bool ArocoHardware::send_null_command_()
{
  memset(sended_frame_data_.data(), 0, 8);
  return send_data_(FRONT_WHEEL_LINEAR_SPEEDS_COMMAND_ID) &&
         send_data_(REAR_WHEEL_LINEAR_SPEEDS_COMMAND_ID) &&
         send_data_(FRONT_STEERING_ANGLE_COMMAND_ID) &&
         send_data_(REAR_STEERING_ANGLE_COMMAND_ID);
}

//-----------------------------------------------------------------------------
bool ArocoHardware::send_data_(uint32_t id)
{
  try {
    drivers::socketcan::CanId can_id(id, 0, 8);
    can_sender_.send(sended_frame_data_.data(), 8, can_id, TIMEOUT);
    return true;
  } catch (drivers::socketcan::SocketCanTimeout & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ArocoHardware"),
      "Send can data" << std::hex << id << " : timeout");
  } catch (std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(
      rclcpp::get_logger("ArocoHardware"),
      "Send can data" << std::hex << id << " : " << e.what());
  }
  return false;
}

//-----------------------------------------------------------------------------
void ArocoHardware::encode_odo_data_(
  float data1,
  float data2)
{
  std::memcpy(&sended_frame_data_[0], &data1, sizeof(data1));
  std::memcpy(&sended_frame_data_[4], &data2, sizeof(data2));
}

//-----------------------------------------------------------------------------
bool ArocoHardware::send_front_wheel_speeds_()
{
  encode_odo_data_(
    front_left_wheel_linear_speed_command_,
    front_right_wheel_linear_speed_command_);
  return send_data_(FRONT_WHEEL_LINEAR_SPEEDS_COMMAND_ID);
}

//-----------------------------------------------------------------------------
bool ArocoHardware::send_rear_wheel_speeds_()
{
  encode_odo_data_(rear_left_wheel_linear_speed_command_, rear_right_wheel_linear_speed_command_);
  return send_data_(REAR_WHEEL_LINEAR_SPEEDS_COMMAND_ID);
}

//-----------------------------------------------------------------------------
bool ArocoHardware::send_front_steering_angle_()
{
  encode_odo_data_(front_axle_steering_angle_command_, front_axle_steering_angle_command_);
  return send_data_(FRONT_STEERING_ANGLE_COMMAND_ID);
}

//-----------------------------------------------------------------------------
bool ArocoHardware::send_rear_steering_angle_()
{
  encode_odo_data_(rear_axle_steering_angle_command_, rear_axle_steering_angle_command_);
  return send_data_(FRONT_STEERING_ANGLE_COMMAND_ID);
}

//-----------------------------------------------------------------------------
bool ArocoHardware::send_start_()
{
  sended_frame_data_[0] = 1;   // Demande départ mode autonome
  return send_data_(START_STOP_ID);
}

//-----------------------------------------------------------------------------
void ArocoHardware::decode_odo_data_(
  std::atomic<float> & data1,
  std::atomic<float> & data2)
{
  data1.store(reinterpret_cast<float *>(received_frame_data_.data())[0]);
  data2.store(reinterpret_cast<float *>(received_frame_data_.data())[1]);
}

//-----------------------------------------------------------------------------
void ArocoHardware::decode_front_wheel_speeds_()
{
  decode_odo_data_(
    front_left_wheel_linear_speed_measure_,
    front_right_wheel_linear_speed_measure_);

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("ArocoHardware"),
    "ID = " << FRONT_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID <<
      " front speeds : left " << front_left_wheel_linear_speed_measure_ <<
      " , right " << front_right_wheel_linear_speed_measure_);
}

//-----------------------------------------------------------------------------
void ArocoHardware::decode_rear_wheel_speeds_()
{
  decode_odo_data_(
    rear_left_wheel_linear_speed_measure_,
    rear_right_wheel_linear_speed_measure_);

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("ArocoHardware"),
    "ID = " << REAR_WHEEL_LINEAR_SPEEDS_MEASUREMENT_ID <<
      " rear speeds : left " << rear_left_wheel_linear_speed_measure_ <<
      " , right " << rear_right_wheel_linear_speed_measure_);
}

//-----------------------------------------------------------------------------
void ArocoHardware::decode_steering_angles_()
{
  decode_odo_data_(
    rear_axle_steering_angle_measure_,
    front_axle_steering_angle_measure_);

  RCLCPP_DEBUG_STREAM(
    rclcpp::get_logger("ArocoHardware"),
    "ID = " << STEERING_ANGLES_MEASUREMENT_ID <<
      " steering angles : front " << front_axle_steering_angle_measure_ <<
      " , rear " << rear_axle_steering_angle_measure_);
}

//-----------------------------------------------------------------------------
void ArocoHardware::start_can_receiver_thread_()
{
  can_receiver_thread_run_ = true;
  can_receiver_thread_ = std::make_unique<std::thread>(&ArocoHardware::receive_data_, this);
}

//-----------------------------------------------------------------------------
void ArocoHardware::stop_can_receiver_thread_()
{
  can_receiver_thread_run_ = false;
  if (can_receiver_thread_->joinable()) {
    can_receiver_thread_->join();
  }
}

//-----------------------------------------------------------------------------
bool ArocoHardware::is_drive_enable_() const
{
  float speed_measure = 0.25 * (front_left_wheel_linear_speed_measure_ +
    front_right_wheel_linear_speed_measure_ +
    rear_left_wheel_linear_speed_measure_ +
    rear_right_wheel_linear_speed_measure_);

  float speed_command = 0.25 * (front_left_wheel_linear_speed_command_ +
    front_right_wheel_linear_speed_command_ +
    rear_left_wheel_linear_speed_command_ +
    rear_right_wheel_linear_speed_command_);

  return !(std::abs(front_axle_steering_angle_measure_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(front_axle_steering_angle_command_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(rear_axle_steering_angle_measure_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(rear_axle_steering_angle_command_) < WHEEL_STEERING_ANGLE_EPSILON &&
         std::abs(speed_measure) < WHEEL_LINEAR_SPEED_EPSILON &&
         std::abs(speed_command) < WHEEL_LINEAR_SPEED_EPSILON);
}

#ifndef NDEBUG
//-----------------------------------------------------------------------------
void ArocoHardware::open_log_file_()
{
  debug_file_.open(
    std::string("aroco.dat").c_str(),
    std::fstream::in | std::fstream::out | std::fstream::trunc);
}
//-----------------------------------------------------------------------------
void ArocoHardware::write_log_header_()
{
  if (debug_file_.is_open()) {
    debug_file_ << " time, ";
    debug_file_ << " FLS, " << " FRS, ";
    debug_file_ << " RLS, " << " RRS, ";
    debug_file_ << " FSA, " << " RSA, ";
    debug_file_ << " FLS_cmd, " << " FRS_cmd, ";
    debug_file_ << " RLS_cmd, " << " RRS_cmd, ";
    debug_file_ << " FSA_cmd, " << " RSA_cmd, " << "\n";
  }
}

//-----------------------------------------------------------------------------
void ArocoHardware::write_log_data_()
{
  if (debug_file_.is_open()) {
    auto now = std::chrono::system_clock::now();
    auto now_ns = std::chrono::time_point_cast<std::chrono::nanoseconds>(now);

    debug_file_ << std::setprecision(10);
    debug_file_ << now_ns.time_since_epoch().count();
    debug_file_ << front_left_wheel_linear_speed_measure_ << " " <<
      front_right_wheel_linear_speed_measure_ << " ";
    debug_file_ << rear_left_wheel_linear_speed_measure_ << " " <<
      rear_right_wheel_linear_speed_measure_ << " ";
    debug_file_ << front_axle_steering_angle_measure_ << " " <<
      rear_axle_steering_angle_measure_ <<
      " ";
    debug_file_ << front_left_wheel_linear_speed_command_ << " " <<
      front_right_wheel_linear_speed_command_ << " ";
    debug_file_ << rear_left_wheel_linear_speed_command_ << " " <<
      rear_right_wheel_linear_speed_command_ << " ";
    debug_file_ << front_axle_steering_angle_command_ << " " <<
      rear_axle_steering_angle_command_ <<
      " /n";
  }
}
#endif

}  // namespace romea

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ArocoHardware, hardware_interface::SystemInterface)
