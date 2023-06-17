#ifndef OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_
#define OMNI_DRIVE_CONTROLLER__OMNI_DRIVE_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

#include "omni_drive_controller/robot_description.hpp"

// optional but recommended
//#include "omni_drive_controller/visibility_control.h"

namespace omni_drive_controller
{
class OmniDriveController : public controller_interface::ControllerInterface
{
    public:
        OmniDriveController();

        // some public variables declared in our controller class to override functions by state
        controller_interface::return_type init(const std::string &controller_name) override;
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;
        controller_interface::InterfaceConfiguration state_interface_configuration()const override;
        // override variables for the state of controller
        controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &previous_state) override;
        controller_interface::return_type update() override;
    
        ~OmniDriveController();

    protected:
        // structure to for storing velocity commands and states of the omni_robot
        struct RimHandle
        {
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> velocity_state;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_command;
        };

        // vector to store rim_name param
        std::vector<std::string> rim_names_;
        // vector to store wheel handle param
        std::vector<RimHandle> registered_rim_names_;
        
        // robot physical paramters from robot_description library
        RobotParams robot_params_{0.03, 0.25};
    
        // using stamped or unstamped param variable
        bool use_stamped_vel_= true;

        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr odometry_transform_publisher_ = nullptr;

        // Timeout to consider cmd_vel commands old
        std::chrono::milliseconds cmd_vel_timeout_{500};

        // variable for subscribers to the topic type geometry_msgs/msg/Twist
        bool subscriber_is_active = false;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_cmd_subscriber = nullptr;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_cmd_unstamped_ = nullptr;

        double publish_rate_ = 50.0;
        rclcpp::Duration publish_period_{0, 0};
        rclcpp::Time previous_publish_timestamp_{0};

    private:
        // functions to return velocity callbacks from the topics /cmd_vel
        void velocityCommandStampedCallback(const geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel);
        void velocityCommandUnstampedCallback(const geometry_msgs::msg::Twist::SharedPtr cmd_vel);

        geometry_msgs::msg::TwistStamped::SharedPtr cmd_vel_;

};

}

#endif