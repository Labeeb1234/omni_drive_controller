#ifndef OMNI_DRIVE_CONTROLLER__KINEMATICS_HPP_
#define OMNI_DRIVE_CONTROLLER__KINEMATICS_HPP_

#include <vector>
#include <memory>
#include <utility>


#include <omni_drive_controller/robot_description.hpp>

namespace omni_drive_controller
{

constexpr int OMNI_ROBOT_MAX_WHEELS = 4;

class kinematics
{
    public:
        explicit kinematics(RobotParams robot_params);
        kinematics();
        ~kinematics();

        // inverse kinematics
        std::vector<double> getRimVelocity(RobotVelocity vel);

        void setRobotParams(RobotParams robot_params);
        
        
    private:
        void initializeParams();
        RobotParams robot_params_;
        std::vector<double> rim_velocity_;
};  


}



#endif