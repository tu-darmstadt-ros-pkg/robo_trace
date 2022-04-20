/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/modes/replay/service/player.hpp"
// MongoCXX
#include <mongocxx/client.hpp>
// Project
#include "robo_trace/storage/connector.hpp"
#include <ros/ros.h>


namespace robo_trace::replay {

RoboTraceReplayService::RoboTraceReplayService(ros::NodeHandle& system_node_handle) 
: PlayerBase(system_node_handle), m_configured(false), m_paused(true), m_flushed(false) {
    // 
}

RoboTraceReplayService::~RoboTraceReplayService() = default;

void RoboTraceReplayService::initialize() {

    /*
        Modify some options to fit with service mode.
    */

    m_time_manager.setTimePublishFrequency(10.0); 

    /*
        Create and advertise services.
    */ 
   
    m_service_configure = m_system_node_handle.advertiseService("configure", &RoboTraceReplayService::onConfigure, this);
    m_service_playback_toggle = m_system_node_handle.advertiseService("toggle", &RoboTraceReplayService::onPlaybackToggle, this);
    m_service_set_playback_time = m_system_node_handle.advertiseService("set_time", &RoboTraceReplayService::onSetPlaybackTime, this);
    m_service_set_playback_speed = m_system_node_handle.advertiseService("set_speed", &RoboTraceReplayService::onSetPlaybackSpeed, this);

}

bool RoboTraceReplayService::onConfigure(robo_trace_msgs::SetConfiguration::Request& request, robo_trace_msgs::SetConfiguration::Response& response) {

    /*
        Clear anything that maybe was already loaded.
    */
    
    m_configured = false;
    m_time_start = 0;

    m_options_connection->m_database_name = request.name;

    m_message_publishers.clear();
    m_publishing_queue = std::priority_queue<MessagePublisher::Ptr, std::vector<MessagePublisher::Ptr>, MessagePublisherComparator>();

    /*
        Fetch the start time of the recording.
    */

    try {
        mongocxx::pool::entry client = robo_trace::store::Connector::instance().getClient();
        (*client).list_databases();
    } catch(std::exception& e) {
        ROS_INFO_STREAM("Failed connecting to database.");
        response.success = false;
        response.message = "Failed connecting to database. Does the requested recording exist?";
        return true;
    }

    const std::optional<double> recording_start_time = getRecordingStartTime();

    if (!recording_start_time) {
        response.success = false;
        response.message = "No topics to play back.";
        return true;
    }

    m_time_start = response.start = recording_start_time.value();

    if (!m_system_node_handle.ok()) {
        response.success = false;
        response.message = "System is shutting down.";
        return true;
    }

    const std::optional<double> recording_end_time = getLastMessageTime();

    if (!recording_start_time) {
        throw std::runtime_error("Found start time, but no end time?");
    }

    m_time_end = response.end = recording_end_time.value();
    ROS_INFO_STREAM("Start: " << std::fixed << m_time_start << " End: " << m_time_end << " Delta: " << (m_time_end - m_time_start));
    /*
        Setup the publishers.
    */ 
    
    setup(std::nullopt, std::nullopt);

    if (!m_system_node_handle.ok()) {
        response.success = false;
        response.message = "System is shutting down.";
        return true;
    }
  
     // Wait until all publishers have finished buffering.
    buffer();

    // Sort in the publishers for timely orderd playback.
    for (MessagePublisher::Ptr publisher : m_message_publishers) {
        
        if (publisher->isCompleted()) {
            continue;
        } else {
            m_publishing_queue.push(publisher);
        }
    }
    
    if (m_message_publishers.empty()) {
        response.success = false;
        response.message = "No topics to play back.";
        return true;
    }

    response.success = true;
    response.message = "Setup completed.";

    m_configured = true;
    
    return true;

}

bool RoboTraceReplayService::onPlaybackToggle(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {

    bool playing = request.data;
    ROS_INFO_STREAM("Playback toggle: " << std::to_string(playing));

    if (playing == m_paused) {
        response.success = false;
        response.message = std::string("Playback already in requested mode.");
    } else {

        m_paused = playing;

        if (!m_paused) {

            ros::WallTime now_wt = ros::WallTime::now();
            m_time_manager.setStartTimeTranslated(ros::Time(now_wt.sec, now_wt.nsec));

            const MessagePublisher::Ptr message_publisher = m_publishing_queue.top();        
            const ros::Time start_time(message_publisher->getNextPublicationTime().value()); 
            m_time_manager.setStartTimeReal(start_time);
            
        }

        response.success = true;
        response.message = std::string("Playback is now running: ") + std::to_string(playing);
        
    }

    return true;

}

bool RoboTraceReplayService::onSetPlaybackTime(robo_trace_msgs::SetFloat::Request& request, robo_trace_msgs::SetFloat::Response& response) {

    const float time = request.data;
    ROS_INFO_STREAM("Time adjustment to " << std::fixed << time << " requested");

    if (time <= 0) {
        response.success = false;
        response.message = std::string("Playback time must be a positive number.");
    } else {

        m_publishing_queue = std::priority_queue<MessagePublisher::Ptr, std::vector<MessagePublisher::Ptr>, MessagePublisherComparator>();
        m_flushed = true;

        const double adjusted_start_time = m_time_start + time;
        m_time_manager.setStartTimeReal(ros::Time(adjusted_start_time));

        const ros::WallTime now_wt = ros::WallTime::now();
        m_time_manager.setStartTimeTranslated(ros::Time(now_wt.sec, now_wt.nsec));

        for (const MessagePublisher::Ptr& publisher : m_message_publishers) {
            publisher->reset(time);
        }

        response.success = true;
        response.message = std::string("Playback time is now at: ") + std::to_string(time);
        
    }

    return true;
   
}

  
bool RoboTraceReplayService::onSetPlaybackSpeed(robo_trace_msgs::SetFloat::Request& request, robo_trace_msgs::SetFloat::Response& response) {

    float scale = request.data;

    if (scale <= 0) {
        response.success = false;
        response.message = std::string("Playback time scale must be a positive number.");
    } else {

        m_time_manager.setTimeScale(scale);

        response.success = true;
        response.message = std::string("Playback time scale is now at: ") + std::to_string(scale);
        
    }

    return true;
 
}

void RoboTraceReplayService::run() {

    const ros::WallDuration time_step_size(0.5);

    while(m_system_node_handle.ok()) {

        if (m_flushed) {
            ROS_INFO_STREAM("Wait...");
            bool some_publishers_still_buffering = std::any_of(std::begin(m_message_publishers), std::end(m_message_publishers),
                [](const MessagePublisher::Ptr& publisher) {
                    return publisher->isBlocked();
                }
            );

            if (some_publishers_still_buffering) {
                time_step_size.sleep();
                ros::spinOnce();
               
            } else {

                for (const MessagePublisher::Ptr& publisher : m_message_publishers)  {
                    if (publisher->isCompleted()) {
                        continue;
                    } else {
                        m_publishing_queue.push(publisher);
                    }
                }

                ROS_INFO_STREAM("Go...");
                m_flushed = false;
        
            }

            continue; 

        }

        // Paused
        if (m_paused || !m_configured || m_publishing_queue.empty() || m_flushed) {
            time_step_size.sleep();
            ros::spinOnce();

        // Running
        } else {

            const MessagePublisher::Ptr message_publisher = m_publishing_queue.top();
            m_publishing_queue.pop();

            if (message_publisher->isCompleted()) {
                continue;
            }
            
            const double time = message_publisher->getNextPublicationTime().value();
            const ros::Time time_original(time); 
            m_time_manager.setExecutionHorizon(time_original);
       
            ros::spinOnce();

            while(!m_time_manager.isHorizonReached()) {
                m_time_manager.run(time_step_size);
                ros::spinOnce();
            } 
          
            message_publisher->publish();
            
            buffer();

            if (message_publisher->isCompleted()) {
                continue;
            }

            m_publishing_queue.push(message_publisher);

        }

    }

}





}