#include <vector>
#include <memory>
#include <cmath>
#include <utility>

#include <omni_drive_controller/kinematics.hpp>
#include <omni_drive_controller/omni_drive_controller.hpp>


namespace omni_drive_controller
{
        kinematics::kinematics(RobotParams robot_params) : robot_params_(robot_params)
        {
            this->initializeParams();

        }

        kinematics::kinematics()
        {
            this->initializeParams();
        }

        void kinematics::setRobotParams(RobotParams robot_params)
        {
            this->robot_params_ = robot_params;
            this->initializeParams();
        }

        void kinematics::initializeParams()
        {
            rim_velocity_.reserve(OMNI_ROBOT_MAX_WHEELS);
            rim_velocity_ = {0, 0, 0, 0};
        }

        // inverse kinematics of the 4-wheel-omni_drive_robot

        std::vector<double> getRimVelocity(RobotVelocity vel)
        {
            double v_x_des = vel.u;
            double v_y_des = vel.v;
            double omega_des = vel.r;
            
            rim_velocity_[0] = v_y_des + ((robot_params_.wheel_separation)/2)*omega_des;
            rim_velocity_[1] = -v_x_des + ((robot_params_.wheel_separation)/2)*omega_des;
            rim_velocity_[2] = -v_y_des +  ((robot_params_.wheel_separation)/2)*omega_des;
            rim_velocity_[3] = v_x_des + ((robot_params_.wheel_separation)/2)*omega_des;

            return rim_velocity_;

        }

        kinematics::~kinematics()


}