#ifndef AROCO_HARDWARE_HPP_
#define AROCO_HARDWARE_HPP_

#include "romea_mobile_base_hardware/hardware_system_interface.hpp"
#include <rclcpp/macros.hpp>

namespace romea
{

class ArocoHardware : public HardwareSystemInterface2AS4WD
{
public:

  RCLCPP_SHARED_PTR_DEFINITIONS(ArocoHardware);

  ArocoHardware();

  virtual hardware_interface::return_type read() override;

  virtual hardware_interface::return_type write() override;

  virtual hardware_interface::return_type connect() override;

  virtual hardware_interface::return_type disconnect() override;

};

}

#endif
