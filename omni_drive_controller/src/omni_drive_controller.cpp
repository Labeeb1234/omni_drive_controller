#include "omni_drive_controller/omni_drive_controller.hpp"

#include <chrono> 
#include <cmath>
#include <exception>

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "tf2/LinearMath/Quaternion.h"


namespace
{   
    // topic names
    constexpr auto DEFAULT_COMMAND_TOPIC = "~/cmd_vel";
    constexpr auto DEFAULT_COMMAND_UNSTAMPED_TOPIC = "~/cmd_vel_unstamped";
    constexpr auto DEFAULT_COMMAND_OUT_TOPIC = "~/cmd_vel_out";
    constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
    constexpr auto WHEELS_QUANTITY = 4; 

}

namespace omni_drive_controller
{
    using namespace std::chrono_literals;  
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;
    using hardware_interface::HW_IF_POSITION;
    using hardware_interface::HW_IF_VELOCITY;
    using lifecycle_msgs::msg::State;
    using std::placeholders::_1;

    OmniDriveController::OmniDriveController()
        : controller_interface::ControllerInterface()
        , cmd_vel_(std::make_shared<geometry_msgs::msg::TwistStamped>()) {}
    
    controller_interface::return_type OmniDriveController::init(
        const std::string &controller_name) 
        {
            //initialize lifecycle node
            auto res = ControllerInterface::init(controller_name);
            if(res != controller_interface::return_type::OK)
            {
                return res;
            }

            try
            {
                auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
                auto_declare<int>("velocity_rolling_window_size", 10);
                auto_declare<bool>("use_stamped_vel", use_stamped_vel_);
            }
            catch(const std::exception& e)
            {
                fprintf(stderr, "Exception during init stage with message: %s\n", e.what());
                return controller_interface::return_type::ERROR;
                
            }
            return controller_interface::return_type::ERROR;
            
        }
        // looping through all wheels and checking command _interface_type ?
        InterfaceConfiguration OmniDriveController::command_interface_configuration() const
        {
            std::vector<std::string> conf_names;
            for(const auto &joint_name: rim_names_)
            {
                conf_names.push_back(joint_name+ "/" + HW_IF_VELOCITY);
            }
            return {interface_configuration_type::INDIVIDUAL, conf_names};
        }

        // looping through all wheels and checking state_interface_type ?


}

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  omni_drive_controllers::OmniDriveController, controller_interface::ControllerInterface)












