#include "my_robot_hardware_interface/my_robot_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ros2_control_demo_hardware
{

// // on_init() method to parse and fetch data from the robot's 
// // URDF/XACRO description.
// CallbackReturn RRBotSystemPositionOnlyHardware::on_init(
//   const hardware_interface::HardwareInfo & info)

hardware_interface::return_type RRBotSystemPositionOnlyHardware::configure(
  const hardware_interface::HardwareInfo & info)
{

  if (configure_default(info) != hardware_interface::return_type::OK)
    return hardware_interface::return_type::ERROR;

  // if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  //   return CallbackReturn::ERROR;

  // START: This part here is for exemplary purposes - Please do not copy to your production code
  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_slowdown_ = stod(info_.hardware_parameters["example_param_hw_slowdown"]);
  
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  hw_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      // return CallbackReturn::ERROR;
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      // return CallbackReturn::ERROR;
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      // return CallbackReturn::ERROR;
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      // return CallbackReturn::ERROR;
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

// // The on_configure() method is used to initiate communication
// // with the hardware and in order to ensure that the hardware states can be read.
// CallbackReturn RRBotSystemPositionOnlyHardware::on_configure(
//     const rclcpp_lifecycle::State &previous_state) 
// {
//   // START: This part here is for exemplary purposes - Please do not copy to
//   // your production code

//   // prevent unused variable warning
//   auto prev_state = previous_state;
//   RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
//               "Configuring ...please wait...");

//   for (int i = 0; i < hw_start_sec_; i++) {
//     rclcpp::sleep_for(std::chrono::seconds(1));
//     RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
//                 "%.1f seconds left...", hw_start_sec_ - i);
//   }
//   // END: This part here is for exemplary purposes - Please do not copy to your
//   // production code

//   // reset values always when configuring hardware
//   for (uint i = 0; i < hw_states_.size(); i++) {
//     hw_states_[i] = 0;
//     hw_commands_[i] = 0;
//   }

//   RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"),
//               "Successfully configured!");

//   return CallbackReturn::SUCCESS;
// }

hardware_interface::return_type RRBotSystemPositionOnlyHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Starting ...please wait...");

  for (auto i = 0; i <= hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (uint i = 0; i < hw_states_.size(); i++) {
    hw_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  // // set some default values
  // for (auto i = 0u; i < hw_positions_.size(); i++)
  // {
  //   if (std::isnan(hw_positions_[i]))
  //   {
  //     hw_positions_[i] = 0;
  //     hw_velocities_[i] = 0;
  //     hw_commands_[i] = 0;
  //   }
  // }

  status_ = hardware_interface::status::STARTED;
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type RRBotSystemPositionOnlyHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Stopping ...please wait...");

  for (auto i = 0; i <= hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "System successfully stopped!");

  return hardware_interface::return_type::OK;
}

// The export_state_interfaces() method is used 
// to define the interfaces that your hardware offers.
std::vector<hardware_interface::StateInterface>
RRBotSystemPositionOnlyHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // This code cycles through all the joints retrieved from the configuration file
  // and sets hardware_interface::HW_IF_POSITION as the only state available
  // from the hardware device.
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RRBotSystemPositionOnlyHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // you create a new vector command_interfaces and then pass in what type of
  // commands are accepted by the hardware. 
  // In this example, you tell ROS2_control that your hardware only accepts
  // position commands
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

// // This method runs the sequence of commands that will power up your hardware
// // to enable movement.
// CallbackReturn RRBotSystemPositionOnlyHardware::on_activate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   // START: This part here is for exemplary purposes - Please do not copy it to your production code
//   RCLCPP_INFO(
//     rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Activating ...please wait...");

//   for (int i = 0; i < hw_start_sec_; i++)
//   {
//     rclcpp::sleep_for(std::chrono::seconds(1));
//     RCLCPP_INFO(
//       rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "%.1f seconds left...",
//       hw_start_sec_ - i);
//   }
//   // END: This part here is for exemplary purposes - Please do not copy to your production code

//   // command and state should be equal when starting
//   for (uint i = 0; i < hw_states_.size(); i++)
//     hw_commands_[i] = hw_states_[i];

//   RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully activated!");

//   return CallbackReturn::SUCCESS;
// }

// CallbackReturn RRBotSystemPositionOnlyHardware::on_deactivate(
//   const rclcpp_lifecycle::State & /*previous_state*/)
// {
//   // START: This part here is for exemplary purposes - Please do not copy to your production code
//   RCLCPP_INFO(
//     rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Deactivating ...please wait...");

//   // The method encloses the chain of processes required to stop the hardware resource
//   // and perform a graceful shutdown and cleanup.
//   for (int i = 0; i < hw_stop_sec_; i++)
//   {
//     rclcpp::sleep_for(std::chrono::seconds(1));
//     RCLCPP_INFO(
//       rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "%.1f seconds left...",
//       hw_stop_sec_ - i);
//   }

//   RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully deactivated!");
//   // END: This part here is for exemplary purposes - Please do not copy to your production code

//   return CallbackReturn::SUCCESS;
// }

// The read() method is to get the states from the hardware and 
// store them to internal variables defined in export_state_interfaces(). 
// The main controller loop then uses those values to do its work.
hardware_interface::return_type RRBotSystemPositionOnlyHardware::read()
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Reading...");

  for (uint i = 0; i < hw_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_states_[i] = hw_states_[i] + (hw_commands_[i] - hw_states_[i]) / hw_slowdown_;
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Got state %.5f for joint %d!",
      hw_states_[i], i);
  }
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

// The write() function is called in the control loop after the 
// computation of the control command for producing actuation signals 
// for the hardware.
hardware_interface::return_type RRBotSystemPositionOnlyHardware::write()
{
  // START: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Writing...");

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Got command %.5f for joint %d!",
      hw_commands_[i], i);
  }
  RCLCPP_INFO(
    rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully written!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

} // namespace ros2_control_demo_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::RRBotSystemPositionOnlyHardware, hardware_interface::SystemInterface)