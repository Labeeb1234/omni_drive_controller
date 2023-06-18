#include <chrono> 
#include <cmath>
#include <exception>
#include <utility>
#include <vector>
#include <queue>
#include <memory>


#include "controller_interface/controller_interface.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"

#include "omni_drive_controller/omni_drive_controller.hpp"



namespace
{
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
    constexpr auto DEFAULT_UNSTAMPED_COMMAND_TOPIC = "~/cmd_vel_unstamped";
    constexpr auto DEFAULT_COMMAND_OUTPUT_TOPIC = "~/cmd_vel_out";
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "~/tf";
    constexpr auto WHEEL_QUANTITY = 4;
}


namespace omni_drive_controller
{
    using namespace std::chrono_literals;
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_VELOCITY;
    using hardware_interface::HW_IF_POSITION;
    using lifecycle_msgs::msg::State;
    using std::placeholders::_1;

    OmniDriveController::OmniDriveController() 
        : controller_interface::ControllerInterface()
        , cmd_vel_(std::make_shared<geometry_msgs::msg::TwistStamped>()) {}
    
    controller_interface::return_type init(const std::string &controller_name)
    {
        // lifecycle node initializing
        auto ret = controller_interface::init(controller_name);;
        if(ret  != controller_interface::return_type::OK)
        {
            return ret;
        }

        try
        {
            auto_declare<std::vector<std::string>>("rim_names", robot_params_.rim_names);
            auto_declare<double>("rim_radius", robot_params_.rim_radius);
            auto_declare<double>("wheel_separation", robot_params_.wheel_separation);
            auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count()/1000);
            auto_declare<bool>("use_stamped_vel", use_stamped_vel_);

        }
        catch(const std::exception& e)
        {
            fprintf(stderr, "Exception thrown in the initalizing stage with messgae: %s\n", e.what());
            return controller_interface::return_type::ERROR;
            
        }

        return controller_interface::return_type::OK;
        
    }

    InterfaceConfiguration OmniDriveController::command_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for(const auto &joint_name : rim_names_)
        {
            conf_names.push_back(joint_name+ "/" + HW_IF_VELOCITY);
        }
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    
    }

    InterfaceConfiguration OmniDriveController::state_interface_configuration() const
    {
        std::vector<std::string> conf_names;
        for(const auto &joint_name : rim_names_)
        {
            conf_names.push_back(joint_name+ "/" + HW_IF_VELOCITY);

        }
    
        return {interface_configuration_type::INDIVIDUAL, conf_names};
    }

    // configuring the robot parametrs for the loaded omni_drive_controller
    CallbackReturn  OmniDriveController::on_configure(const rclcpp_lifecycle::State  &previous_state)
    {
        auto logger = get_node()->get_logger();

        RCLCPP_DEBUG(logger, "Called on_configure. Previous state was %s", previous_state.label());
        rim_names_ = get_node()->get_parameter("rim_names").as_string_array();

        if(rim_names_.size() != WHEEL_QUANTITY)
        {
            RCLCPP_ERROR(logger, "The number wheels/rims [%zu] and required wheel/rim number [%zu] are different", rim_names_.size(), WHEEL_QUANTITY);
            return CallbackReturn::ERROR;
        }
        if(rim_names_.empty())
        {
            RCLCPP_ERROR(logger, "Wheel/rim name parametres missing");
            return CallbackReturn::ERROR;
        }


    }

    robot_params_.rim_radius = get_node()->get_parameter("rim_radius").as_double();
    robot_params_.wheel_separation = get_node()->get_parameter("wheel_separation").as_double();

    cmd_vel_timeout_ = 





}