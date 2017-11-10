/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SafeController.hpp"

using namespace skid4_control;

SafeController::SafeController(std::string const& name)
    : SafeControllerBase(name)
{
    _cmd_timeout.set(base::Time::fromSeconds(0.1));
}

SafeController::SafeController(std::string const& name, RTT::ExecutionEngine* engine)
    : SafeControllerBase(name, engine)
{
    _cmd_timeout.set(base::Time::fromSeconds(0.1));
}

SafeController::~SafeController()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See SafeController.hpp for more detailed
// documentation about them.

bool SafeController::configureHook()
{
    if (! SafeControllerBase::configureHook())
        return false;
    
    m_radius = _wheel_radius.get();
    m_trackRadius = _track_width.get() / 2;
    m_cmd_timeout = base::Timeout(_cmd_timeout.value());
    if (m_radius <= 0 || m_trackRadius <= 0)
    {
        RTT::log(RTT::Error) << "wrong radius and/or track_width parameters" << RTT::endlog();
        return false;
    }
    joint_status.resize(m_jointCmd.size());
    
    m_jointCmd.getElementByName(_front_left_name.get());
    
    frontLeftIdx = m_jointCmd.mapNameToIndex(_front_left_name.get());
    frontRightIdx = m_jointCmd.mapNameToIndex(_front_right_name.get());
    rearLeftIdx = m_jointCmd.mapNameToIndex(_rear_left_name.get());
    rearRightIdx = m_jointCmd.mapNameToIndex(_rear_right_name.get());
    
    if(_max_torque.get() < _minLowerWheelTorque.get())
    {
        std::cout << "max torque must be bigger minLowerWheelTorque " << std::endl;
        return false;
    }
    
    return true;    
}
bool SafeController::startHook()
{
    if (! SafeControllerBase::startHook())
        return false;
    
    gotBody2Gravity = false;
    
    _body2imu.registerUpdateCallback([&] (const base::Time &t) {
	base::samples::RigidBodyState body2Imu;
	if(!_body2imu.get(t, body2Imu))
	  return;
	body2Gravity = base::removeYaw(body2Imu.orientation);
	gotBody2Gravity = true;
    } );
    
    return true;
}

void SafeController::synchronizeSpeeds(size_t mainwheelIdx, double wantedSpeed, const std::vector<size_t> &allWheelIdx)
{
    float min_speed = std::abs(joint_status.getElementByName(m_jointCmd.names[mainwheelIdx]).speed);

    for(std::vector<size_t>::const_iterator it = allWheelIdx.begin(); it != allWheelIdx.end();it++)
    {
        base::JointState &s(m_jointCmd.elements[*it]);
        s.speed = wantedSpeed;
        s.effort = maxTorque;
        if(std::abs(min_speed) < std::abs(wantedSpeed) && *it != mainwheelIdx)
        {
            float new_speed = std::copysign(min_speed, wantedSpeed);
            //make sure the other wheels tuns at least, if your main wheel stalls
            if(std::abs(new_speed) < std::abs(wantedSpeed) * 0.2)
                s.speed = wantedSpeed * 0.2;
            else
                s.speed = new_speed;
        }
    }
}



void SafeController::updateHook()
{
    SafeController::updateHook();

    // This is the user's command
    base::commands::Motion2D cmd_in;
    RTT::FlowStatus status = _motion_command.readNewest(cmd_in);
    if (status == RTT::NoData)
    {
        cmd_in.translation = 0;
        cmd_in.rotation    = 0;
    }
    else if(status == RTT::NewData)
        m_cmd_timeout.restart();
    else if(status == RTT::OldData && m_cmd_timeout.elapsed()) // check for input timeout
    {
        cmd_in.translation = 0;
        cmd_in.rotation    = 0;
    }

    RTT::FlowStatus joint_status_fstatus = _status_samples.readNewest(joint_status);
    if (joint_status_fstatus == RTT::NoData)
    {
        RTT::log(RTT::Warning) << "Waiting for status samples." << RTT::endlog();
        return;
    }

    double fwd_velocity = cmd_in.translation / m_radius;
    double differential = cmd_in.rotation * m_trackRadius / m_radius;

    double bodyPitch = base::getPitch(body2Gravity);
    
    double minLowerWheelTorque = _minLowerWheelTorque.get();
    double lowTorquePitchLimit = _lowTorquePitchLimit.get();

    double lowerWheelTorque = minLowerWheelTorque;
    
    if(fabs(bodyPitch) < lowTorquePitchLimit)
    {
        lowerWheelTorque = minLowerWheelTorque + (_max_torque.get() - minLowerWheelTorque) * fabs(bodyPitch) / lowTorquePitchLimit;
    }    
    
    if(bodyPitch > 0)
    {
        //if we drive upwards, the rear wheels are the master wheels
        synchronizeSpeeds(rearLeftIdx, fwd_velocity - differential, m_leftIndexes);
        synchronizeSpeeds(rearRightIdx, fwd_velocity + differential, m_rightIndexes);
        
        m_jointCmd[rearLeftIdx].effort = lowerWheelTorque;
        m_jointCmd[rearRightIdx].effort = lowerWheelTorque;
        m_jointCmd[frontLeftIdx].effort = maxTorque;
        m_jointCmd[frontRightIdx].effort = maxTorque;
        
    }
    else
    {
        //downhil, front wheel is the master wheel
        synchronizeSpeeds(frontLeftIdx, fwd_velocity - differential, m_leftIndexes);
        synchronizeSpeeds(frontRightIdx, fwd_velocity + differential, m_rightIndexes);

        m_jointCmd[rearLeftIdx].effort = maxTorque;
        m_jointCmd[rearRightIdx].effort = maxTorque;
        m_jointCmd[frontLeftIdx].effort = lowerWheelTorque;
        m_jointCmd[frontRightIdx].effort = lowerWheelTorque;
    }

    m_jointCmd.time = base::Time::now();

    _command.write(m_jointCmd);
}

void SafeController::errorHook()
{
    SafeControllerBase::errorHook();
}
void SafeController::stopHook()
{
    SafeControllerBase::stopHook();
}
void SafeController::cleanupHook()
{
    SafeControllerBase::cleanupHook();
}
