/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "SimpleController.hpp"
#include <base/commands/Motion2D.hpp>

using namespace skid4_control;

SimpleController::SimpleController(std::string const& name)
    : SimpleControllerBase(name)
{
    _cmd_timeout.set(base::Time::fromSeconds(0.1));
}

SimpleController::SimpleController(std::string const& name, RTT::ExecutionEngine* engine)
    : SimpleControllerBase(name, engine)
{
    _cmd_timeout.set(base::Time::fromSeconds(0.1));
}


bool SimpleController::configureHook()
{
    if(!SimpleControllerBase::configureHook())
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

    return true;
}

void SimpleController::synchronizeSpeeds(const std::string &mainWheel, size_t mainwheelIdx, double wantedSpeed, const std::vector<size_t> &allWheelIdx)
{
    float min_speed = std::abs(joint_status.getElementByName(mainWheel).speed);

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


void SimpleController::updateHook()
{
    SimpleControllerBase::updateHook();

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

    base::samples::Joints joint_status_in;
    RTT::FlowStatus joint_status_fstatus = _status_samples.readNewest(joint_status_in);
    if (joint_status_fstatus == RTT::NoData)
    {
        RTT::log(RTT::Warning) << "Waiting for status samples." << RTT::endlog();
        return;
    }
    else if(joint_status_fstatus == RTT::NewData)
    {
        double delta_t = (joint_status_in.time - joint_status.time).toSeconds();
	if(delta_t > 0.05)
	{
            joint_status = joint_status_in;
	}
        _status_samples_debug.write(joint_status);
    }

    double fwd_velocity = cmd_in.translation / m_radius;
    double differential = cmd_in.rotation * m_trackRadius / m_radius;

    //the 1 is a big HACK we assume the rear wheel ist last in the name vector
    synchronizeSpeeds(m_LeftWheelNames[1], m_leftIndexes[1], fwd_velocity - differential, m_leftIndexes);
    synchronizeSpeeds(m_RightWheelNames[1], m_rightIndexes[1], fwd_velocity + differential, m_rightIndexes);

    m_jointCmd.time = base::Time::now();

    _command.write(m_jointCmd);
}
