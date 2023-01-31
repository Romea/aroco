// Copyright 2022 INRAE, French National Research Institute for Agriculture, Food and Environment
// Add license

#ifndef AROCO_HARDWARE__AROCO_HARDWARE_HPP_
#define AROCO_HARDWARE__AROCO_HARDWARE_HPP_

// romea
#include <romea_mobile_base_hardware/hardware_system_interface.hpp>

// ros
#include <ros2_socketcan/socket_can_receiver.hpp>
#include <ros2_socketcan/socket_can_sender.hpp>
#include <rclcpp/macros.hpp>

// std
#include <array>
#include <atomic>
#include <memory>
#include <thread>
#include <fstream>

namespace romea
{

class ArocoHardware : public HardwareSystemInterface2AS4WD
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArocoHardware);

  ArocoHardware();
  #if ROS_DISTRO == ROS_GALACTIC
  virtual hardware_interface::return_type read();

  virtual hardware_interface::return_type write();
#else
  virtual hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period);

  virtual hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period);
#endif

private:
  hardware_interface::return_type connect_() override;

  hardware_interface::return_type disconnect_() override;

  hardware_interface::return_type load_info_(
    const hardware_interface::HardwareInfo & hardware_info) override;


  void encode_odo_data_(
    float data1,
    float data2);

  void decode_odo_data_(
    std::atomic<float> & data1,
    std::atomic<float> & data2);


  bool send_data_(uint32_t id);

  bool send_front_wheel_speeds_();

  bool send_rear_wheel_speeds_();

  bool send_front_steering_angle_();

  bool send_rear_steering_angle_();

  bool send_command_();

  bool send_null_command_();

  bool send_start_();


  void receive_data_();

  void decode_front_wheel_speeds_();

  void decode_rear_wheel_speeds_();

  void decode_steering_angles_();

  bool is_drive_enable_() const;


  void start_can_receiver_thread_();

  void stop_can_receiver_thread_();

  void get_hardware_command_();

  void set_hardware_state_();

#ifndef NDEBUG
  void open_log_file_();
  void write_log_header_();
  void write_log_data_();
#endif

private:
  std::unique_ptr<std::thread> can_receiver_thread_;
  std::atomic<bool> can_receiver_thread_run_;

  drivers::socketcan::SocketCanSender can_sender_;
  drivers::socketcan::SocketCanReceiver can_receiver_;
  std::array<uint8_t, 8> sended_frame_data_;
  std::array<uint8_t, 8> received_frame_data_;

  float front_wheel_radius_;
  float rear_wheel_radius_;

  std::atomic<float> front_axle_steering_angle_measure_;
  std::atomic<float> rear_axle_steering_angle_measure_;
  std::atomic<float> front_left_wheel_linear_speed_measure_;
  std::atomic<float> front_right_wheel_linear_speed_measure_;
  std::atomic<float> rear_left_wheel_linear_speed_measure_;
  std::atomic<float> rear_right_wheel_linear_speed_measure_;

  float front_axle_steering_angle_command_;
  float rear_axle_steering_angle_command_;
  float front_left_wheel_linear_speed_command_;
  float front_right_wheel_linear_speed_command_;
  float rear_left_wheel_linear_speed_command_;
  float rear_right_wheel_linear_speed_command_;

#ifndef NDEBUG
  std::fstream debug_file_;
#endif
};

}  // namespace romea

#endif  // AROCO_HARDWARE__AROCO_HARDWARE_HPP_
