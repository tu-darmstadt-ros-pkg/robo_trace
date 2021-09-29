// Base
#include "robo_trace/modes/capture/persistor.hpp"
// Project
#include "robo_trace/util/smart_ptr_conversions.hpp"
#include "robo_trace/storage/container.hpp"
#include "robo_trace/processing/context.hpp"


namespace robo_trace {
    

TopicPersistor::TopicPersistor(const RecorderOptions::ConstPtr options_recorder, const std::vector<ProcessingStage::Ptr>& pipeline, ros::NodeHandle& node_handle, const std::string& topic)
:  m_topic(topic), m_options_recorder(options_recorder), m_node_handle(node_handle), m_pipeline(pipeline) {
    //
}

TopicPersistor::~TopicPersistor() = default;


const std::string& TopicPersistor::getTopic() const {
    return m_topic;
}

const std::vector<ProcessingStage::Ptr>& TopicPersistor::getPipeline() const {
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

    DataContainer::Ptr metadata_container = std::make_shared<DataContainer>();
    
    // Nice... Technically, we could get all the topic here from the connectio header...
    // ROS_INFO_STREAM("Def: " << event.getConnectionHeader()["message_definition"]);

    const double ingress_time = event.getReceiptTime().toSec();
    metadata_container->append("time", ingress_time);
    
    // Maybe there are multiple nodes publishing to a topic.
    const std::string& origin_node = event.getPublisherName();
    metadata_container->append("origin", origin_node);
   
    // Create containers for message and associated metadata.
    const ros_babel_fish::BabelFishMessage::ConstPtr msg = event.getConstMessage(); 

    ProcessingContext::Ptr context = std::make_shared<ProcessingContext>(metadata_container);
    context->setUnserializedMessage(msg);
    
    for (const ProcessingStage::Ptr& processing_stage : m_pipeline) {
        
        processing_stage->process(context);

        if (context->isTerminated()) {
            return;
        }

    }
   
}

} 
