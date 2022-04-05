// Base
#include "robo_trace/modes/capture/recorder.hpp"  
// Std
#include <algorithm>
// Boost
#include <boost/regex.hpp>
// Ros
#include <ros/network.h>
#include <ros/xmlrpc_manager.h>
#include <xmlrpcpp/XmlRpc.h>
// Project
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/persistors/direct.hpp"
#include "robo_trace/storage/persistors/batch.hpp"
#include "robo_trace/processing/modules/storage/forward.hpp"


namespace robo_trace::capture {

Recorder::Recorder(ros::NodeHandle& system_node_handle) 
: m_system_node_handle(system_node_handle) {
    //
}

Recorder::~Recorder() = default;

void Recorder::initialize(int argc, char** argv) {

    if (!m_system_node_handle.ok()) {
        return;
    }

    /*
        Load the program options.
    */

    m_option_recorder = std::make_shared<robo_trace::capture::Options>();
    m_options_connection = std::make_shared<robo_trace::store::Options>();

    robo_trace::util::Options::load({m_option_recorder, m_options_connection}, m_system_node_handle, argc, argv);
    
    /*
        Initialize the persistor spinner.
    */

    m_persistor_callback_queue = std::make_unique<ros::CallbackQueue>();
    
    m_persistor_spinner = std::make_unique<ros::AsyncSpinner>(THRAD_COUNT_IO, m_persistor_callback_queue.get());
    m_persistor_spinner->start();

    /*
        Initialize the MongoDB connector.
    */

    robo_trace::store::Connector::instance().configure(m_options_connection);
  
    /*
        Initialize the persistor for to the metadata collection.
    */

    m_persistor_metadata = std::make_shared<robo_trace::store::DirectPersistor>(m_options_connection->m_database_name, m_options_connection->m_collection_name_summary);

#ifdef PERSISTOR_USE_UNIQUE_BUCKET
    /*

    */

    m_stream_handler = std::make_shared<robo_trace::store::StreamHandler>(m_options_connection->m_database_name, PERSISTOR_GLOBAL_BUCKET_NAME);
#endif

    /*
        Initialize the pipeline constructor.
    */
    
    m_pipeline_constructor.initialize(m_options_connection, m_system_node_handle);

    /*
        Kick-Off recording feedback loops.
    */

    m_check_topics_timer = m_system_node_handle.createTimer(ros::Duration(m_option_recorder->m_capture_topic_check_period), &Recorder::onCheckForNewTopics, this);
   
}

void Recorder::terminate(int signal) {
    (void) signal;
    
    ROS_INFO_STREAM("Terminating recorder.");

    /*
        Stop all current recording related acticity.
    */

    m_check_topics_timer.stop();

    for (const auto& [topic, persistor] : m_persistors) {
        persistor->stop();
    }

    /*
        Flush the message persitors.
    */

    ROS_INFO_STREAM("Uploading pending messages.");

    for (const auto& [topic, persistor] : m_persistors) {
        persistor->flush();
    }

    /*
        Wait for the job queue to be fully processed.
    */

    ros::Rate check_rate(0.5);

    while(!m_persistor_callback_queue->empty()) {
        ROS_INFO_STREAM(" - Working.");
        check_rate.sleep();
    }

    /*
        Shutdown remaining components.
    */
  
    m_persistor_spinner->stop();
    m_persistors.clear();

    ROS_INFO_STREAM("Bye.");

    /*
        Delegate to ROS.
    */

    ros::requestShutdown();

}


void Recorder::onCheckForNewTopics(const ros::TimerEvent& event) {

    /*
        Check master for new topics. It may be the case that we simply record 
        everything here, have some specific topics, or a regex match.
    */

    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);

    for (ros::master::V_TopicInfo::iterator it = topic_list.begin() ; it != topic_list.end(); it++) {

        const ros::master::TopicInfo& topic_info = *it;
        const std::string& topic_name = topic_info.name;

        /*
            Should we record this topic?
        */

        if (!isToBeRecorded(topic_name)) {
            continue;
        }

        record(topic_info);

    }

    /*
        Query for a specific node's topics if desired.
    */

    if (m_option_recorder->m_capture_all || !m_option_recorder->m_capture_node) {
        return;
    }

    XmlRpc::XmlRpcValue request_node_address_lookup;
    request_node_address_lookup[0] = ros::this_node::getName();
    request_node_address_lookup[1] = m_option_recorder->m_capture_node_name;
    
    XmlRpc::XmlRpcValue response_node_address_lookup;
    XmlRpc::XmlRpcValue payload_node_address_lookup;

    if (!ros::master::execute("lookupNode", request_node_address_lookup, response_node_address_lookup, payload_node_address_lookup, true)) {
        return;
    }

    std::string peer_host;
    uint32_t peer_port;

    if (!ros::network::splitURI(static_cast<std::string>(response_node_address_lookup[2]), peer_host, peer_port)) {
        ROS_ERROR("Bad xml-rpc URI trying to inspect node at: [%s]", static_cast<std::string>(response_node_address_lookup[2]).c_str());
        return;
    } 
    
    XmlRpc::XmlRpcValue request_get_subscriptions;
    request_get_subscriptions[0] = ros::this_node::getName();

    XmlRpc::XmlRpcValue response_get_subscriptions;
    
    XmlRpc::XmlRpcClient client(peer_host.c_str(), peer_port, "/");
    client.execute("getSubscriptions", request_get_subscriptions, response_get_subscriptions);
    
    if (client.isFault() || !response_get_subscriptions.valid() || response_get_subscriptions.size() == 0 || static_cast<int>(response_get_subscriptions[0]) != 1) {
        ROS_ERROR("Node at: [%s] failed to return subscriptions.", static_cast<std::string>(response_get_subscriptions[2]).c_str());
        return;
    }

    for(int i = 0; i < response_get_subscriptions[2].size(); i++) {
        
        // Index 0 holds the topic name, and index 1 the message type. 
        //  -> http://wiki.ros.org/ROS/Slave_API
        const std::string& topic_name = response_get_subscriptions[2][i][0];
        const std::string& message_type = response_get_subscriptions[2][i][1];

        if (!isToBeRecorded(topic_name, true)) {
            continue;
        }
        
        ros::master::TopicInfo topic_info(topic_name, message_type);
        // Check record.
        record(topic_info);
    }

}

void Recorder::record(const ros::master::TopicInfo& topic_info) {

    const std::string& topic = topic_info.name;
    const std::string& message_type = topic_info.datatype;

    ROS_INFO_STREAM("Start recording for topic: " << topic);

    /*
        Build up topic summary container.
    */

    robo_trace::store::Container::Ptr topic_recording_summary = std::make_shared<robo_trace::store::Container>();
    topic_recording_summary->append("topic", topic);
    topic_recording_summary->append("message_type", message_type);
    // Taking the time here should be okay. It is only used by playback to determine the approx. overall start time.
    topic_recording_summary->append("start_time", ros::Time::now().toSec());

    /*
        Construct the pipeline.
    */
    
    std::vector<robo_trace::processing::Processor::Ptr> pipeline_stages = m_pipeline_constructor.construct(robo_trace::processing::Mode::CAPTURE, topic_recording_summary, topic);
    
    /*
        Construct and add the persistor module to the pipeline. 
    */

    std::string message_type_adjusted = message_type;
    std::replace(message_type_adjusted.begin(), message_type_adjusted.end(), '/', '_');

    int buffer_size = 0;
    ros::param::param<int>("/robo_trace/capture/upload/buffering/" + message_type_adjusted, buffer_size, 32);
    ROS_INFO_STREAM(" Buffer size is " << std::to_string(buffer_size) << " (Topic: " << topic << ")");

    robo_trace::store::Persistor::Ptr writeback_persistor = std::make_shared<robo_trace::store::BatchPersistor>(
        // database
        m_options_connection->m_database_name, 
        // collection
        topic,
        // callback_queue
        m_persistor_callback_queue.get(),
        // buffer_size
        buffer_size
    );
    writeback_persistor->setIndex("metadata.time", PERSISTOR_TREAT_TIME_AS_UNIQUE);

    // The storage stage is added manually.
    const robo_trace::processing::Processor::Ptr storage_stage = std::make_shared<robo_trace::processing::StorageForwardProcessor>(
                // persistor
                writeback_persistor,
                // metadata
                topic_recording_summary
            );
    pipeline_stages.push_back(storage_stage);

    /*
        Construct the stream handler.
    */
#ifdef PERSISTOR_USE_UNIQUE_BUCKET
    const robo_trace::store::StreamHandler::Ptr& stream_handler = m_stream_handler; 
#else

   const robo_trace::store::StreamHandler::Ptr = std::make_shared<robo_trace::store::StreamHandler>(
       // database
       m_options_connection->m_database_name, 
       // bucket
       topic
    );

#endif

    /*

    */

    // The persistor will handle subscription and pipeline invocation.
    MessageStreamRecorder::Ptr topic_persistor = std::make_shared<MessageStreamRecorder>(
        // options_recorder
        m_option_recorder,
        // pipeline
        pipeline_stages,
        // persistor
        writeback_persistor,
        // stream_handler
        stream_handler,
        // node_handle 
        m_system_node_handle, 
        // topic
        topic
    );

    m_persistors.insert({{topic, topic_persistor}});

    /*
        Kick of recording.
    */

    topic_persistor->start();    

    /*
        Store the metadata for this topic
    */
    
    bsoncxx::builder::basic::document topic_summary_mongo_builder{};
    topic_recording_summary->serialize(topic_summary_mongo_builder);

    bsoncxx::document::value topic_summary_mongo_entry = topic_summary_mongo_builder.extract();
    m_persistor_metadata->store(topic_summary_mongo_entry);
    
    
}

bool Recorder::isToBeRecorded(const std::string& topic, const bool is_capture_node) {

    /*
        We are already recoding that topic, so we don't want to record it twice.
    */

    if (m_persistors.find(topic) != m_persistors.end()) {
        return false;
    }

    /*
        Is the topic from the recorder itself?
    */

    if (topic.find(ROBO_TRACE_NODE_NAME) != std::string::npos) {
        return false;
    }

    /*
        Check if this topic is on the blacklist.
    */

    if (m_option_recorder->m_capture_exclude_by_regex && boost::regex_match(topic, m_option_recorder->m_capture_exclude_regex)) {
        return false;
    }

    /*
        We are recording everything.
    */

    if(m_option_recorder->m_capture_all || (m_option_recorder->m_capture_node && is_capture_node)) {
        return true;
    }

    /*
        The topic list is treated as regex. Check if any matches.
    */

    if (m_option_recorder->m_capture_topics_by_regex) {
        // Treat the topics as regular expressions
        return std::any_of(std::begin(m_option_recorder->m_capture_topics), std::end(m_option_recorder->m_capture_topics),
                [&topic] (const std::string& regex_str) {
                    boost::regex e(regex_str);
                    boost::smatch what;               
                    return boost::regex_match(topic, what, e, boost::match_extra);
                }
            );
    }

    /*
        Check if any topic in the inclusive list matches.
    */

    return std::find(std::begin(m_option_recorder->m_capture_topics), std::end(m_option_recorder->m_capture_topics), topic) != std::end(m_option_recorder->m_capture_topics);

}




}
