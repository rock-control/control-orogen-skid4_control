#include "FourWheelController.hpp"

using namespace skid4_control;

FourWheelController::FourWheelController(std::string const& name)
    : FourWheelControllerBase(name)
{
    _cmd_timeout.set(base::Time::fromSeconds(0.1));
}

FourWheelController::FourWheelController(std::string const& name, RTT::ExecutionEngine* engine)
    : FourWheelControllerBase(name, engine)
{
    _cmd_timeout.set(base::Time::fromSeconds(0.1));
}

FourWheelController::~FourWheelController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See FourWheelController.hpp for more detailed
// documentation about them.

bool FourWheelController::configureHook()
{
    if (! FourWheelControllerBase::configureHook())
        return false;

    m_wheel_radius = _wheel_radius.get();
    m_trackWidth = _track_width.get();
    m_axisDist = _axis_distance.get();
    m_cmd_timeout = base::Timeout(_cmd_timeout.value());
    if (m_wheel_radius <= 0 || m_trackWidth <= 0 || m_axisDist <= 0)
    {
        RTT::log(RTT::Error) << "wrong radius and/or track_width parameters" << RTT::endlog();
        return false;
    }
    return true;
}
bool FourWheelController::startHook()
{
    if (! FourWheelControllerBase::startHook())
        return false;
    return true;
}

double FourWheelController::getRadPerSecondForDistance(double dist) const
{
    //short of dist / (m_wheel_radius * 2 * M_PI) * 2 * M_PI
    return dist / m_wheel_radius;
}


void FourWheelController::updateHook()
{
    FourWheelControllerBase::updateHook();
    
    // This is the user's command
    base::commands::Motion2D cmd_in;
    RTT::FlowStatus status = _motion_command.readNewest(cmd_in);
    if(status == RTT::NewData)
        m_cmd_timeout.restart();
    else if((status == RTT::OldData && m_cmd_timeout.elapsed())
            || (status == RTT::NoData)) // check for input timeout
    {
        cmd_in.translation = 0;
        cmd_in.rotation    = 0;
    }
    
    double distLeft = cmd_in.translation;
    double distRight = cmd_in.translation;
    
    if(cmd_in.rotation != 0.0)
    {
        double radius = cmd_in.translation / cmd_in.rotation;
    
//        double radiusLeft  = Eigen::Vector2d(radius - m_axisDist / 2.0, m_trackWidth / 2.0).norm();
//        double radiusRight = Eigen::Vector2d(radius + m_axisDist / 2.0, m_trackWidth / 2.0).norm();
        double radiusLeft  = radius - m_trackWidth / 2.0;
        double radiusRight = radius + m_trackWidth / 2.0;

        //compute how many radian we need to travel on the main
        //circle in one second
        // 1s * cmd_in.translation / radius == cmd_in.rotation
        double angle = cmd_in.rotation;
        
        distLeft = angle * radiusLeft;
        distRight = angle * radiusRight;
    }
    
    double radPerSecondLeft = getRadPerSecondForDistance(distLeft);
    double radPerSecondRight = getRadPerSecondForDistance(distRight);
    
    for(std::vector<size_t>::const_iterator it = m_leftIndexes.begin(); it != m_leftIndexes.end();it++)
    {
        m_jointCmd.elements[*it].speed = radPerSecondLeft;
    }
    for(std::vector<size_t>::const_iterator it = m_rightIndexes.begin(); it != m_rightIndexes.end();it++)
    {
        m_jointCmd.elements[*it].speed = radPerSecondRight;
    }
    
    m_jointCmd.time = base::Time::now();
    
    _command.write(m_jointCmd);
}
void FourWheelController::errorHook()
{
    FourWheelControllerBase::errorHook();
}
void FourWheelController::stopHook()
{
    FourWheelControllerBase::stopHook();
}
void FourWheelController::cleanupHook()
{
    FourWheelControllerBase::cleanupHook();
}
