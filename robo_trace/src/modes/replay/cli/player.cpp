/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/modes/replay/cli/player.hpp"
// Std
#include <vector>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/mode.hpp"
#include "robo_trace/processing/processor.hpp"


namespace robo_trace::replay {

RoboTracePlayer::RoboTracePlayer(ros::NodeHandle& system_node_handle)
: PlayerBase(system_node_handle) {
    //   
}

RoboTracePlayer::~RoboTracePlayer() = default;

bool RoboTracePlayer::isCompleted() const {
    return std::all_of(std::begin(m_message_publishers), std::end(m_message_publishers),
        [](const MessagePublisher::Ptr& publisher) {
            return publisher->isCompleted();
        }
    );
}

bool RoboTracePlayer::isToBePlayedBack(const std::string& topic) const {

    if (m_options_player->m_replay_topics.empty()) {
        return true;
    }

    return std::find(
                // first
                std::begin(m_options_player->m_replay_topics), 
                // last
                std::end(m_options_player->m_replay_topics), 
                // value
                topic
           ) != std::end(m_options_player->m_replay_topics);

}

void RoboTracePlayer::initialize() {

    /*
        Build base a query for the messages. Currently this query only constrains time.

        TODO: Allow for injecting other constrains.
    */

    std::optional<double> time_start = {};
    std::optional<double> time_end = {};
    
    if (m_options_player->m_time_start || m_options_player->m_time_duration) {

        std::optional<double> recording_start_time = getRecordingStartTime();

        ros::spinOnce();

        if (!m_system_node_handle.ok()) {
            return;
        }

        // Should only be a nullopt if no topic matches or the recording is empty.
        if (!recording_start_time) {
            std::cout << "No topics to play back." << std::endl; exit(0);
        }

        double time_adjusted_start = recording_start_time.value_or(0) + m_options_player->m_time_start.value_or(0);

        if (m_options_player->m_time_start) {
            time_start = time_adjusted_start;
        }

        if (m_options_player->m_time_duration) {
            time_end = time_adjusted_start + m_options_player->m_time_duration.value();
        }

    }

    setup(time_start, time_end);

    if (m_message_publishers.empty()) {
        std::cout << " - No topics to play back!" << std::endl; exit(0);
    }

    /*
        Sleep for some duration after advertising the topics. 
    */

    if (!m_options_player->m_wait_duration_after_advertise.isZero()) {
        std::cout << "Waiting " << m_options_player->m_wait_duration_after_advertise.toSec() << " seconds after advertising topics..." << std::flush;   
        m_options_player->m_wait_duration_after_advertise.sleep();
        std::cout << " done." << std::endl;
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

}

void RoboTracePlayer::run() {

    if (m_options_player->m_publish_clock_frequency) {
        m_time_manager.setTimePublishFrequency(m_options_player->m_publish_clock_frequency.value());
    }

    ROS_INFO_STREAM("Starting playing: " << m_publishing_queue.top()->getNextPublicationTime().value_or(0));

    if(!m_publishing_queue.empty()) {
        
        const MessagePublisher::Ptr message_publisher = m_publishing_queue.top();
        
        const ros::Time start_time(message_publisher->getNextPublicationTime().value()); 
        m_time_manager.setStartTimeReal(start_time);

    }

    ros::WallTime now_wt = ros::WallTime::now();
    m_time_manager.setStartTimeTranslated(ros::Time(now_wt.sec, now_wt.nsec));

    ROS_INFO_STREAM("Starting publishing");

    while(!m_publishing_queue.empty() && m_system_node_handle.ok()) {

        const MessagePublisher::Ptr message_publisher = m_publishing_queue.top();
        m_publishing_queue.pop();

        const double time = message_publisher->getNextPublicationTime().value();
        const ros::Time time_original(time); 
        m_time_manager.setExecutionHorizon(time_original);
        
        ros::spinOnce();

        while(!m_time_manager.isHorizonReached()) {
            m_time_manager.run(ros::WallDuration(.1));
            ros::spinOnce();
        } 

        message_publisher->publish();
        
        if (message_publisher->isCompleted()) {
            continue;
        }

        m_publishing_queue.push(message_publisher);

    }

}


}