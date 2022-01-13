// Base
#include "robo_trace/modes/capture/persistor.hpp"
// Project
#include "robo_trace/util/smart_ptr_conversions.hpp"
#include "robo_trace/storage/container.hpp"
#include "robo_trace/processing/context.hpp"

#ifdef RECORDING_SIGNAL_PIPELINE_PASS
#include "std_msgs/Empty.h"
#endif


namespace robo_trace::capture {  

TopicPersistor::TopicPersistor(const Options::ConstPtr options_recorder, const std::vector<robo_trace::processing::Processor::Ptr>& pipeline, ros::NodeHandle& node_handle, const std::string& topic)
:  m_topic(topic), m_options_recorder(options_recorder), m_node_handle(node_handle), m_pipeline(pipeline) {
    
#ifdef RECORDING_SIGNAL_PIPELINE_PASS
    const std::string signal_pipeline_pass_topic_name = ros::names::append(RECORDING_SIGNAL_PIPELINE_PASS_TOPIC_PREFIX, topic);
    m_publisher_signal_pipeline_pass = node_handle.advertise<std_msgs::Empty>(signal_pipeline_pass_topic_name, RECORDING_SIGNAL_PIPELINE_PASS_TOPIC_QUEUE_SIZE);
#endif

}

TopicPersistor::~TopicPersistor() = default;

const std::string& TopicPersistor::getTopic() const {
    return m_topic;
}

const std::vector<robo_trace::processing::Processor::Ptr>& TopicPersistor::getPipeline() const {
    return m_pipeline;
}

ros::NodeHandle& TopicPersistor::getNodeHandle() {
    return m_node_handle;
}

void TopicPersistor::start() {
    
    if (m_subscriber_active) {
        return;
    }

    m_subscriber = m_node_handle.subscribe(
        // topic
        m_topic, 
        // queue_size
        m_options_recorder->m_capture_subscriber_queue_size, 
        // fp
        &TopicPersistor::process, 
        // obj
        this,
        // transport_hints
        m_options_recorder->m_capture_subscriber_transport_hints
    );
    
    m_subscriber_active = true;
}

void TopicPersistor::stop() {
    
    if (!m_subscriber_active) {
        return;
    }

    m_subscriber.shutdown();
    m_subscriber_active = false;
    
}

void TopicPersistor::process(const ros::MessageEvent<const ros_babel_fish::BabelFishMessage>& event) {
    
    m_messages_received_local += 1;
    // m_messages_received_total += 1;

    robo_trace::store::Container::Ptr metadata_container = std::make_shared<robo_trace::store::Container>();
    
    // Nice... Technically, we could get all the topic here from the connectio header...
    // ROS_INFO_STREAM("Def: " << event.getConnectionHeader()["message_definition"]);

    const double ingress_time = event.getReceiptTime().toSec();
    metadata_container->append("time", ingress_time);
    
    // Maybe there are multiple nodes publishing to a topic.
    const std::string& origin_node = event.getPublisherName();
    metadata_container->append("origin", origin_node);
   
    // Create containers for message and associated metadata.
    const ros_babel_fish::BabelFishMessage::ConstPtr msg = event.getConstMessage(); 

    robo_trace::processing::Context::Ptr context = std::make_shared<robo_trace::processing::Context>(metadata_container);
    context->setRosMessage(msg);
    
    for (const robo_trace::processing::Processor::Ptr& processing_stage : m_pipeline) {
        
        processing_stage->process(context);

        if (context->isTerminated()) {
            return;
        }

    }

#ifdef RECORDING_SIGNAL_PIPELINE_PASS
    const std_msgs::Empty pipeline_pass_signal;
    m_publisher_signal_pipeline_pass.publish(pipeline_pass_signal);
#endif
   
}

} 
