#include "aroco_hardware/aroco_hardware.hpp"
#include <rclcpp/rclcpp.hpp>

namespace romea {

//-----------------------------------------------------------------------------
ArocoHardware::ArocoHardware():
  HardwareSystemInterface2AS4WD()
{

}

//-----------------------------------------------------------------------------
hardware_interface::return_type ArocoHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("ArocoHardware"), "Read data from robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type ArocoHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("ArocoHardware"), "Send command to robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type ArocoHardware::connect()
{
  RCLCPP_INFO(rclcpp::get_logger("ArocoHardware"), "Init communication with robot");
  return hardware_interface::return_type::OK;
}

//-----------------------------------------------------------------------------
hardware_interface::return_type ArocoHardware::disconnect()
{
  RCLCPP_INFO(rclcpp::get_logger("ArocoHardware"), "Close communication with robot");
  return hardware_interface::return_type::OK;
}

};

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(romea::ArocoHardware, hardware_interface::SystemInterface)
