#ifndef OMNI_DRIVE_CONTROLLER__ROBOT_DESCRIPTION_HPP_
#define OMNI_DRIVE_CONTROLLER__ROBOT_DESCRIPTION_HPP_

#define DEG2RAD(deg) (deg*M_PI/180)


//library for robot description
namespace omni_drive_controller
{

    struct RobotParams
    {
        double rim_radius;
        double wheel_separation;
    };

    struct RobotVelocity
    {
        double u;
        double v;
        double r;
    }; 

    struct Pose
    {
        double x;
        double y;
        double theta;
    };

}




#endif