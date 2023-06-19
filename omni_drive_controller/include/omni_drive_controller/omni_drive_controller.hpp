#ifndef OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_
#define OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>
#include <utility>


#include "controller_interface/controller_interface.hpp"
//#include "controller_interface/controller_interface_base.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_msgs/msg/tf_message.hpp"


#include "omni_drive_controller/robot_description.hpp"
#include "omni_drive_controller/kinematics.hpp"

// optional but recommended
//#include "omni_drive_controller/visibility_control.h"

namespace omni_drive_controller
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class OmniDriveController : public controller_interface::ControllerInterface
{
  public:
    OmniDriveController();
    controller_interface::return_type init(
    const std::string &controller_name, const std::string &namespace_ = "",
    const rclcpp::NodeOptions & node_options =
      rclcpp::NodeOptions()
        .allow_undeclared_parameters(true)
        .automatically_declare_parameters_from_overrides(true)) override;
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration &period) override;
    ~OmniDriveController();

  protected:
    struct WheelHandle 
    {
      std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
      std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
    };

    std::vector<std::string> rim_names_;
    std::vector<WheelHandle> registered_rim_handles_;

    // Default parameters for omni_drive_robot
    RobotParams robot_params_{0.03, 0.25};

    bool use_stamped_vel_ = true;
    // declaring object for kinematics class
    kinematics omni_robot_kinematics_;

    // Timeout to consider cmd_vel commands old
    std::chrono::milliseconds cmd_vel_timeout_{500};

    bool subscriber_is_active_ = false;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_subscriber_ = nullptr;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_unstamped_subscriber_ = nullptr;

    rclcpp::Time previous_update_timestamp_{0};

    double publish_rate_{50.0};
    rclcpp::Duration publish_period_{0, 0};
    rclcpp::Time previous_publish_timestamp_{0};

  private:
    void velocityCommandStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel);
    void velocityCommandUnstampedCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);
    geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel_;

};

}  

#endif 