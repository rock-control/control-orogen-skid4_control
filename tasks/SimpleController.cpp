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
        return;

    base::samples::Joints joint_status_in;
    RTT::FlowStatus joint_status_fstatus = _status_samples.readNewest(joint_status_in);
    if (joint_status_fstatus == RTT::NoData)
    {
        RTT::log(RTT::Warning) << "Waiting for status samples." << RTT::endlog();
        return;
    }
    else if(joint_status_fstatus == RTT::NewData)
    {
        for(size_t i = 0; i < joint_status_in.size(); ++i)
        {
            joint_status.names[i] = joint_status_in.names[i];
            if(!base::isUnset(joint_status_in.elements[i].speed))
            {
                joint_status.elements[i].speed = joint_status_in.elements[i].speed;
            }
            else if(!base::isUnset(joint_status_in.elements[i].position))
            {
                if(!base::isUnset(joint_status.elements[i].position))
                {
                    joint_status.elements[i].speed = (joint_status_in.elements[i].position - joint_status.elements[i].position) / (joint_status_in.time - joint_status.time).toSeconds();
                }
                joint_status.elements[i].position = joint_status_in.elements[i].position;
            }
        }
        joint_status.time = joint_status_in.time;
        _status_samples_debug.write(joint_status);
    }

    float min_speed_right = std::numeric_limits< float >::max();
    size_t min_speed_right_idx = -1;
    float min_speed_left = std::numeric_limits< float >::max();
    size_t min_speed_left_idx = -1;
    for(size_t i = 0; i < m_LeftWheelNames.size(); ++i)
    {
        try
        {
            float speed = std::abs(joint_status[m_LeftWheelNames[i]].speed);
            if(!base::isNaN(speed) && speed < min_speed_left);
            {
                min_speed_left_idx = m_leftIndexes[i];
                min_speed_left = speed;
            }
        }
        catch(const base::samples::Joints::InvalidName& e)
        {
            RTT::log(RTT::Debug) << e.what() << RTT::endlog();
        }
    }
    for(size_t i = 0; i < m_RightWheelNames.size(); ++i)
    {
        try
        {
            float speed = std::abs(joint_status[m_RightWheelNames[i]].speed);
            if(!base::isNaN(speed) && speed < min_speed_right);
            {
                min_speed_right_idx = m_rightIndexes[i];
                min_speed_right = speed;
            }
        }
        catch(const base::samples::Joints::InvalidName& e)
        {
            RTT::log(RTT::Debug) << e.what() << RTT::endlog();
        }
    }

    double fwd_velocity = cmd_in.translation / m_radius;
    double differential = cmd_in.rotation * m_trackRadius / m_radius;

    for(std::vector<size_t>::const_iterator it = m_leftIndexes.begin(); it != m_leftIndexes.end();it++)
    {
        base::JointState &s(m_jointCmd.elements[*it]);
        s.speed = fwd_velocity - differential;
        s.effort = maxTorque;
        if(min_speed_left_idx >= 0 && *it != min_speed_left_idx)
        {
            s.speed = std::copysign(min_speed_left, s.speed);
        }
    }
    for(std::vector<size_t>::const_iterator it = m_rightIndexes.begin(); it != m_rightIndexes.end();it++)
    {
        base::JointState &s(m_jointCmd.elements[*it]);
        s.speed = fwd_velocity + differential;
        s.effort = maxTorque;
        if(min_speed_right_idx >= 0 && *it != min_speed_right_idx)
        {
            s.speed = std::copysign(min_speed_right, s.speed);
        }
    }

    m_jointCmd.time = base::Time::now();

    _command.write(m_jointCmd);
}
