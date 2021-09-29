// Base
#include "robo_trace/modes/capture/recorder.hpp"  
// Boost
#include <boost/regex.hpp>
// Ros
#include <ros/network.h>
#include <ros/xmlrpc_manager.h>
#include <xmlrpcpp/XmlRpc.h>
// Project
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/storage/container.hpp"
#include "robo_trace/processing/stage/storage/descriptor.hpp"


namespace robo_trace {

RoboTraceRecorder::RoboTraceRecorder(ros::NodeHandle& system_node_handle) 
: m_system_node_handle(system_node_handle) {
    //
}

RoboTraceRecorder::~RoboTraceRecorder() {

}

void RoboTraceRecorder::initialize(int argc, char** argv) {

    if (!m_system_node_handle.ok()) {
        return;
    }

    /*
        Load the program options.
    */

    m_option_recorder = std::make_shared<RecorderOptions>();
    m_options_connection = std::make_shared<ConnectionOptions>();

    OptionsContainer::load({m_option_recorder, m_options_connection}, m_system_node_handle, argc, argv);

    /*
        Initialize the storage stage.
    */
    
    ConnectionProvider::initialize();
    
    m_storage_stage_descriptor = std::make_shared<StorageStageDescriptor>(m_options_connection, m_system_node_handle);

    /*
        Initialize the pipeline constructor.
    */
    
    m_pipeline_constructor.initialize(m_options_connection, m_system_node_handle);

    /*
        Kick-Off recording feedback loops.
    */

    m_check_topics_timer = m_system_node_handle.createTimer(ros::Duration(m_option_recorder->m_capture_topic_check_period), &RoboTraceRecorder::onCheckForNewTopics, this);
   
}


void RoboTraceRecorder::onCheckForNewTopics(const ros::TimerEvent& event) {

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

void RoboTraceRecorder::record(const ros::master::TopicInfo& topic_info) {

    const std::string& topic = topic_info.name;
    const std::string& message_type = topic_info.datatype;

    ROS_INFO_STREAM("Start recording for topic: " << topic);

    /*
        Build up topic summary container.
    */

    DataContainer::Ptr topic_recording_summary = std::make_shared<DataContainer>();
    topic_recording_summary->append("topic", topic);
    topic_recording_summary->append("message_type", message_type);
    // Taking the time here should be okay. It is only used by playback to determine the approx. overall start time.
    topic_recording_summary->append("start_time", ros::Time::now().toSec());

    /*
        Construct the pipeline.
    */
    
    std::vector<ProcessingStage::Ptr> pipeline_stages = m_pipeline_constructor.construct(ProcessingMode::CAPTURE, topic_recording_summary, topic);
     
    // The storage stage is added manually.
    const ProcessingStage::Ptr storage_stage = m_storage_stage_descriptor->getStage(topic_recording_summary, ProcessingMode::CAPTURE).value();
    pipeline_stages.push_back(storage_stage);
    
    // The persistor will handle subscription and pipeline invocation.
    TopicPersistor::Ptr persistor = std::make_shared<TopicPersistor>(m_option_recorder, pipeline_stages, m_system_node_handle, topic);

    // std::make_pair<std::string, TopicPersistor::Ptr>(std::move(topic), persistor)
    m_persistors.insert({{topic, persistor}});

    /*
        Kick of recording.
    */

    persistor->start();    
    
}

bool RoboTraceRecorder::isToBeRecorded(const std::string& topic, const bool is_capture_node) {

    /*
        We are already recoding that topic, so we don't want to record it twice.
    */

    if (m_persistors.find(topic) != m_persistors.end()) {
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