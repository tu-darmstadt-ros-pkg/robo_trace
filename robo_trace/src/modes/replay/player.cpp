// Base
#include "robo_trace/modes/replay/player.hpp"
// Std
#include <vector>
// MongoDB
#include "robo_trace/config.h"
#include <mongo/client/dbclientinterface.h>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/modes/modes.hpp"
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/query.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/stage/stage.hpp"


namespace robo_trace {

RoboTracePlayer::RoboTracePlayer(ros::NodeHandle& system_node_handle)
: m_system_node_handle(system_node_handle), m_time_manager(system_node_handle) {
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

void RoboTracePlayer::initialize(int argc, char** argv) {

    if (!m_system_node_handle.ok()) {
        return;
    }

    /*
        Load the program options.
    */

    m_options_player = std::make_shared<PlayerOptions>();
    m_options_connection = std::make_shared<ConnectionOptions>();

    OptionsContainer::load({m_options_player, m_options_connection}, m_system_node_handle, argc, argv);

    /*
        Initialize the pipeline constructor.
    */

    m_pipeline_constructor.initialize(m_options_connection, m_system_node_handle);

    /*
        Get a connection to the database.
    */

    ConnectionProvider::initialize();

    const std::shared_ptr<mongo::DBClientConnection> connection = ConnectionProvider::getConnection(m_options_connection); 

    /*
        Build base a query for the messages. Currently this query only constrains time.

        TODO: Allow for injecting other constrains.
    */

    MessageQuery structure_query;

    if (m_options_player->m_time_start || m_options_player->m_time_duration) {

        std::optional<double> recording_start_time = getRecordStartTime(connection);

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
            structure_query.setFieldEqualGreaterThan("metadata.time", time_adjusted_start);
        }

        if (m_options_player->m_time_duration) {
            structure_query.setFieldLessThan("metadata.time", time_adjusted_start + m_options_player->m_time_duration.value());
        }

    }

    mongo::BSONObjBuilder mongo_structure_query_builder;
    //structure_query.serialize(mongo_structure_query_builder);
    // "allowDiskUse" : true
    //mongo_structure_query_builder.append("allowDiskUse", true);
    mongo::BSONObj mongo_structure_query = mongo_structure_query_builder.obj();

    std::cout << "Query: " << mongo_structure_query.jsonString() << std::endl;

    mongo::Query mongo_query(mongo_structure_query);  
    mongo_query.sort("metadata.time", 1);

    /*
        Load topic specific publishers.
    */

    const std::string summary_collection_path = m_options_connection->m_database_name + "." +  m_options_connection->m_summary_collection_name;
    // Fetch everything from the global summary collection.
    std::unique_ptr<mongo::DBClientCursor> recording_summary_cursor = connection->query(
        // ns
        summary_collection_path,
        // query
        mongo_query
    );

    std::cout << "Loading recording information. Topics to be played back are:" << std::endl;

    while (recording_summary_cursor->more()) {

        const mongo::BSONObj topic_recording_info = recording_summary_cursor->next();
        const DataContainer::Ptr topic_data = std::make_shared<DataContainer>(topic_recording_info);

        /*
            Should we playback this topic?
        */

        const std::string topic_name = topic_recording_info["topic"].str();
        std::cout << " - " << topic_name << std::endl;

        if (!isToBePlayedBack(topic_name)) {
            continue;
        }

        /*
            Construct the backward processing pipeline.
        */

        std::vector<ProcessingStage::Ptr> pipeline_stages = m_pipeline_constructor.construct(ProcessingMode::REPLAY, topic_data, topic_name);
        
        /*
            Construct the message loader.
        */ 
     
        const std::string collection_name = topic_recording_info["collection"].str();
        const std::string collection_path = m_options_connection->m_database_name + "." + collection_name;

        MessageLoader::Ptr loader = std::make_shared<MessageLoader>(
            // connector
            m_options_connection,
            // query,
            mongo_query,
            // collection
            collection_path,
            // pipeline
            pipeline_stages,
            // callback_queue
            // TODO: Don't use the global queue here.
            (ros::CallbackQueueInterface*) ros::getGlobalCallbackQueue()
        );
        loader->schedule();

        /*
            Construct the message publishing helper.
        */

        MessagePublisher::Ptr publisher = std::make_shared<MessagePublisher>(
            // fish
            m_babel_fish,
            // handle,
            m_system_node_handle,
            // options
            m_options_player,
            // loader
            loader,
            // data
            topic_data
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

    play();

}

std::optional<double> RoboTracePlayer::getRecordStartTime() {
    return getRecordStartTime(ConnectionProvider::getConnection(m_options_connection));
}

std::optional<double> RoboTracePlayer::getRecordStartTime(const std::shared_ptr<mongo::DBClientConnection> connection) {

    /*
        Query the summary collection.
    */

    mongo::Query mongo_query;  
    mongo_query.sort("start_time", 1);

    const std::string summary_collection_path = m_options_connection->m_database_name + "." +  m_options_connection->m_summary_collection_name;
    // Fetch everything from the global summary collection.
    std::unique_ptr<mongo::DBClientCursor> recording_summary_cursor = connection->query(
        // namespace
        summary_collection_path,
        // query
        mongo_query
        // nToReturn 
        // 1 - We can't do this as some topics may be selected specifically
    );

    /*
        Find the earliest message time, while respecting the topic selection list.
    */

    std::optional<double> start_time = {};

    while (recording_summary_cursor->more()) {

        const mongo::BSONObj topic_recording_info = recording_summary_cursor->next();
        const std::string topic_name = topic_recording_info["topic"].str();

        if (!isToBePlayedBack(topic_name)) {
            continue;
        }

        double topic_start_time = topic_recording_info["start_time"].numberDouble();

        if (!start_time || topic_start_time < start_time.value()) {
            start_time = topic_start_time;
        } 

    }

    return start_time;
}

void RoboTracePlayer::buffer() {

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

void RoboTracePlayer::play() {

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

    while(!m_publishing_queue.empty()) {
   
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