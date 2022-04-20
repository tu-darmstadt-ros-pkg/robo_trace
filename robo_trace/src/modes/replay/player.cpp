/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/modes/replay/player.hpp"
// MongoCXX
#include <bsoncxx/document/view.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
// Project
#include "robo_trace/storage/connector.hpp"


namespace robo_trace::replay {

PlayerBase::PlayerBase(ros::NodeHandle& system_node_handle)
: m_system_node_handle(system_node_handle), m_time_manager(system_node_handle) {
    //
}

PlayerBase::~PlayerBase() = default;

void PlayerBase::initialize(int argc, char** argv) {

    if (!m_system_node_handle.ok()) {
        return;
    }

    /*
        Load the program options.
    */

    m_options_player = std::make_shared<robo_trace::replay::Options>();
      m_options_connection = std::make_shared<robo_trace::store::Options>();

    robo_trace::util::Options::load({m_options_player, m_options_connection}, m_system_node_handle, argc, argv);

    /*
        Initialize the pipeline constructor.
    */

    m_pipeline_constructor.initialize(m_options_connection, m_system_node_handle);

    /*
        Setup the MongoDB driver.
    */

    robo_trace::store::Connector::instance().configure(m_options_connection);

    /*
        Initialize the loader spinner.
    */

    m_loader_callback_queue = std::make_unique<ros::CallbackQueue>();
    // TODO: Option in the config file.
    m_loader_spinner = std::make_unique<ros::AsyncSpinner>(1, m_loader_callback_queue.get());

    m_loader_spinner->start();

    /*
        If we are not in detached mode, play right away.
    */

    initialize();

}

void PlayerBase::initialize() {
    // 
}

void PlayerBase::setup(const std::optional<float> time_start, const std::optional<float> time_end) {

    /*
        Load topic specific publishers.
    */

    mongocxx::pool::entry client = robo_trace::store::Connector::instance().getClient();
    mongocxx::cursor result = (*client)[m_options_connection->m_database_name][m_options_connection->m_collection_name_summary].find({});

    for(mongocxx::cursor::iterator iterator = result.begin(); iterator != result.end(); ++iterator) {

        const bsoncxx::document::view& topic_recording_info = *iterator;
   
        /*
            Should we playback this topic?
        */

        const std::string topic_name = topic_recording_info["topic"].get_utf8().value.to_string();
        
        if (!isToBePlayedBack(topic_name)) {
            continue;
        }
    
        /*
            Construct the backward processing pipeline.
        */

        const robo_trace::store::Container::Ptr topic_data = std::make_shared<robo_trace::store::Container>(topic_recording_info);
        std::vector<robo_trace::processing::Processor::Ptr> pipeline_stages = m_pipeline_constructor.construct(robo_trace::processing::Mode::REPLAY, topic_data, topic_name);
        
        /*
            Construct the message loader.
        */ 
     
        const std::string collection_name = topic_recording_info["collection"].get_utf8().value.to_string();
        
        MessageLoader::Ptr loader = std::make_shared<MessageLoader>(
            // collection
            collection_name,
            // database
            m_options_connection->m_database_name,
            // pipeline
            pipeline_stages,
            // callback_queue
            (ros::CallbackQueueInterface*) m_loader_callback_queue.get(),
            // start_time
            time_start,
            // end_time
            time_end,
            // query,
            std::nullopt
        );
        loader->schedule();

        /*
            Construct the message publishing helper.
        */

        MessagePublisher::Ptr publisher = std::make_shared<MessagePublisher>(
            // loader
            loader,
            // data
            topic_data,
            // fish
            m_babel_fish,
            // handle,
            m_system_node_handle,
            // topic prefix
            m_options_player->m_topic_prefix,
            // topic queue size
            m_options_player->m_topic_queue_size
        );
        m_message_publishers.push_back(std::move(publisher));

        /*
            Give ROS some space to breathe.
        */

        ros::spinOnce();

        if (!m_system_node_handle.ok()) {
            return;
        }

    }

}

bool PlayerBase::isToBePlayedBack(const std::string& topic) const {
    return true;
}

void PlayerBase::buffer() {

    // Whait while the loaders have completed buffering.
    for (;;) {
    
        bool some_publishers_still_buffering = std::any_of(std::begin(m_message_publishers), std::end(m_message_publishers),
                [](const MessagePublisher::Ptr& publisher) {
                    return publisher->isBlocked();
                }
            );

        if (some_publishers_still_buffering) {

            ros::WallDuration(0.1).sleep();
            // Give ROS some room to breathe.
            ros::spinOnce();

            if (!m_system_node_handle.ok()) {
                return;
            }

        } else {
            break;            
        }
 
    }

}

std::optional<double> PlayerBase::getFirstMessageTime() {
    return getMessageTimeLimit(PlayerBase::MessagePosition::First);
}

std::optional<double> PlayerBase::getLastMessageTime() {
    return getMessageTimeLimit(PlayerBase::MessagePosition::Last);
}

std::optional<double> PlayerBase::getMessageTimeLimit(const PlayerBase::MessagePosition& position) {

    mongocxx::pool::entry client = robo_trace::store::Connector::instance().getClient();
    mongocxx::cursor result = (*client)[m_options_connection->m_database_name][m_options_connection->m_collection_name_summary].find({});

    /*
        Find the latest message time.
    */

    std::optional<double> extreme_time = {};

    for(mongocxx::cursor::iterator iterator = result.begin(); iterator != result.end(); ++iterator) {

        const bsoncxx::document::view& topic_recording_info = *iterator;
        const std::string topic_name = topic_recording_info["topic"].get_utf8().value.to_string();

        if (!isToBePlayedBack(topic_name)) {
            continue;
        }

        const std::string collection_name = topic_recording_info["collection"].get_utf8().value.to_string();
        const std::string collection_path = m_options_connection->m_database_name + "." + collection_name;
        
        mongocxx::options::find options_query;

        switch(position) {
            case PlayerBase::MessagePosition::First : {
                options_query.sort(bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp("metadata.time", 1)));
                break;
            }
            case PlayerBase::MessagePosition::Last : {
                options_query.sort(bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp("metadata.time", -1)));
                break;
            }
        }
        
        bsoncxx::stdx::optional<bsoncxx::document::value> result = (*client)[m_options_connection->m_database_name][collection_path].find_one(
            // filter
            {},
            // options
            options_query
        );

        if (!result) {
            continue;
        }

        const bsoncxx::document::view serialized_message = result.value().view();
        const double message_time = serialized_message["metadata"]["time"].get_double();

        switch(position) {
            case PlayerBase::MessagePosition::First : {
                if (!extreme_time || message_time < extreme_time.value()) {
                    extreme_time = message_time;
                } 
                break;
            }
            case PlayerBase::MessagePosition::Last : {
                if (!extreme_time || extreme_time.value() < message_time) {
                    extreme_time = message_time;
                } 
                break;
            }
        }
       

    }

    return extreme_time;

}

std::optional<double> PlayerBase::getRecordingStartTime() {

    

    /*
        Query the summary collection.
    */

    mongocxx::options::find options_query;
    options_query.sort(bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp("metadata.time", 1)));

    mongocxx::pool::entry client = robo_trace::store::Connector::instance().getClient();
    mongocxx::cursor result = (*client)[m_options_connection->m_database_name][m_options_connection->m_collection_name_summary].find(
        // filter
        {},
        // options
        options_query
    );

    /*
        Find the earliest message time.
    */

    std::optional<double> start_time = {};

    for(mongocxx::cursor::iterator iterator = result.begin(); iterator != result.end(); ++iterator) {

        const bsoncxx::document::view& topic_recording_info = *iterator;
        const std::string topic_name = topic_recording_info["topic"].get_utf8().value.to_string();

        double topic_start_time = topic_recording_info["start_time"].get_double();

        if (!start_time || topic_start_time < start_time.value()) {
            start_time = topic_start_time;
        } 

    }

    return start_time;

}

}