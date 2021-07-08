// Base
#include "robo_trace_core/processing/forward/pipeline.hpp"
// Ros
#include "ros/console.h"
// Project
#include "robo_trace_plugin_interface/processing/message.hpp"
#include "robo_trace_plugin_interface/processing/context.hpp"


namespace robo_trace {
    
ForwardProcessingPipeline::ForwardProcessingPipeline(ros_babel_fish::BabelFish& fish, ros::NodeHandle& node_handle, const MessageStore::Ptr& store, const std::string& topic) 
: m_node_handle(node_handle), m_fish(fish), m_topic(topic), m_store(store) {
    //
}

ForwardProcessingPipeline::~ForwardProcessingPipeline() = default;


ros::NodeHandle& ForwardProcessingPipeline::getNodeHandle() {
    return m_node_handle;
}

ros_babel_fish::BabelFish& ForwardProcessingPipeline::getBabelFish() {
    return m_fish;
}

const std::string& ForwardProcessingPipeline::getTopic() const {
    return m_topic;
}

const MessageStore::Ptr& ForwardProcessingPipeline::getStore() const {
    return m_store;
}

std::vector<ProcessingStage::Ptr>& ForwardProcessingPipeline::getStages() {
    return m_stages;
}

const ProcessingStageInvoker::Ptr& ForwardProcessingPipeline::getInvoker() const {
    return m_invoker;
}

void ForwardProcessingPipeline::setInvoker(const ProcessingStageInvoker::Ptr& invoker) {
    m_invoker = invoker;
}

void ForwardProcessingPipeline::start() {
    if (!m_subscriber_active) {
        m_subscriber = m_node_handle.subscribe<ros_babel_fish::BabelFishMessage>(m_topic, 1, &ForwardProcessingPipeline::process, this);
    }
}

void ForwardProcessingPipeline::stop() {
    if (m_subscriber_active) {
        m_subscriber.shutdown();
    }
}

void ForwardProcessingPipeline::process(const ros_babel_fish::BabelFishMessage::ConstPtr& msg) {
    
    // The wrapper ensures consistency between the decoded message and the message stream.
    Message::Ptr message_wrapper = std::make_shared<Message>(msg);
    
    // Store all kinds of stuff that stages might think is necessary
    MetadataContainer::Ptr metadata_container = m_store->getContainer();
 
    MessageProcessingContext::Ptr context = std::make_shared<MessageProcessingContext>(metadata_container, message_wrapper);
    m_invoker->enqueue(context);

    m_store->store(context);

}

} 
