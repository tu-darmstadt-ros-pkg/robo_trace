// Base
#include "robo_trace/modes/replay/time.hpp"
// RosGraph
#include "rosgraph_msgs/Clock.h"


namespace robo_trace {


TimeManager::TimeManager(ros::NodeHandle& node_handle)
: m_node_handle(node_handle),
  m_time_scale(1.0) {
    
    // Disable publications by default.
    setTimePublishFrequency(-1.0);  
    // TODO: Make the topic name configurable maybe?
    m_publication_publisher = m_node_handle.advertise<rosgraph_msgs::Clock>("clock", 1);

}

    
TimeManager::~TimeManager() {

}

bool TimeManager::isHorizonReached() {
    return ros::WallTime::now() > m_horizon_wc;
}

double TimeManager::getTimeScale() const {
    return m_time_scale;
}

void TimeManager::setTimeScale(double time_scale) {
    m_time_scale = time_scale;
}

const ros::Time& TimeManager::getStartTimeReal() const {
    return m_start_time_real;
}

void TimeManager::setStartTimeReal(const ros::Time& time) {
    m_start_time_real = time;
}

const ros::Time& TimeManager::getStartTimeTranslated() const {
    return m_start_time_translated;
}

void TimeManager::setStartTimeTranslated(const ros::Time & time) {
    m_start_time_translated = time;
}

double TimeManager::getTimePublishFrequency() const {
    return m_publication_frequency;
}

void TimeManager::setTimePublishFrequency(double publish_frequency) {
    m_publication_frequency = publish_frequency;
    m_publication_enabled = (publish_frequency > 0);
    m_publication_wall_step.fromSec(1.0 / publish_frequency);
}
       
const ros::Time& TimeManager::getCurrentTime() const {
    return m_current_time;
}

void TimeManager::setCurrentTime(const ros::Time& time) {
    m_current_time = time;
}


const ros::Time& TimeManager::getExecutionHorizon() const {
    return m_horizon;
}

void TimeManager::setExecutionHorizon(const ros::Time& horizon) {

    m_horizon = horizon;

    const ros::Time time_translated = m_start_time_translated + (horizon - m_start_time_real) * (1.0 / m_time_scale);
    m_horizon_wc = ros::WallTime(time_translated.sec, time_translated.nsec);

}
 

void TimeManager::advance(const ros::Duration& duration) {
    m_start_time_translated += duration;
}

void TimeManager::run(const ros::WallDuration& duration) {

    if (m_publication_enabled)  {

        rosgraph_msgs::Clock msg_clock_time;

        ros::WallTime time_current = ros::WallTime::now();
        ros::WallTime time_end = time_current + duration;

        while (time_current < time_end && time_current < m_horizon_wc) {

            ros::WallDuration time_left_till_wc_horizon = m_horizon_wc - ros::WallTime::now();

            ros::Duration duration_till_wc_horizon(time_left_till_wc_horizon.sec, time_left_till_wc_horizon.nsec);
            duration_till_wc_horizon *= m_time_scale;

            m_current_time = m_horizon - duration_till_wc_horizon;
        
            if (m_current_time >= m_horizon)
                m_current_time = m_horizon;

            if (time_current >= m_publication_next_time) {
                
                msg_clock_time.clock = m_current_time;
                m_publication_publisher.publish(msg_clock_time);

                m_publication_next_time = time_current + m_publication_wall_step;

            }

            ros::WallTime time_target = time_end;

            if (time_target > m_horizon_wc) {
                time_target = m_horizon_wc;
            }

            if (time_target > m_publication_next_time) {
                time_target = m_publication_next_time;
            }
            
            ros::WallTime::sleepUntil(time_target);

            time_current = ros::WallTime::now();

        }

    } else {

        ros::WallDuration time_left_till_wc_horizon = m_horizon_wc - ros::WallTime::now();

        ros::Duration duration_till_wc_horizon(time_left_till_wc_horizon.sec, time_left_till_wc_horizon.nsec);
        duration_till_wc_horizon *= m_time_scale;

        m_current_time = m_horizon - duration_till_wc_horizon;
        
        if (m_current_time >= m_horizon) {
            m_current_time = m_horizon;
        }

        ros::WallTime time_target = ros::WallTime::now() + duration;

        if (time_target > m_horizon_wc) {
            time_target = m_horizon_wc;
        }

        ros::WallTime::sleepUntil(time_target);

    }

}
    
void TimeManager::stalled(const ros::WallDuration& duration) {

    if (m_publication_enabled) {

        rosgraph_msgs::Clock msg_clock_time;

        ros::WallTime time_current = ros::WallTime::now();
        ros::WallTime time_end = time_current + duration;

        while (time_current < time_end) {

            if (time_current > m_publication_next_time) {
                
                msg_clock_time.clock = m_current_time;
                m_publication_publisher.publish(msg_clock_time);

                m_publication_next_time = time_current + m_publication_wall_step;
            
            }

            ros::WallTime time_target = time_end;

            // Clip to the next timestamp, where we need to publish time information.
            if (time_target > m_publication_next_time) {
                time_target = m_publication_next_time;
            }

            ros::WallTime::sleepUntil(time_target);

            time_current = ros::WallTime::now();

        }

    } else {
        duration.sleep();
    }

}
    
void TimeManager::step() {

    m_current_time = m_horizon;

    if (!m_publication_enabled) {
        return;
    }

    rosgraph_msgs::Clock msg_clock_time;
    msg_clock_time.clock = m_current_time;

    m_publication_publisher.publish(msg_clock_time);

    m_publication_next_time = ros::WallTime::now() + m_publication_wall_step;
  
}


}