// Base
#include "robo_trace_core/processing/marshalling/descriptor.hpp"
// BabelFish
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
// Project
#include "robo_trace_core/processing/marshalling/serializer.hpp"


namespace robo_trace {

BasicMessageMarshallingStageDescriptor::BasicMessageMarshallingStageDescriptor(const ros::NodeHandle& plugin_namespace) 
: BasicMessageMarshallingStageDescriptor(plugin_namespace, std::make_shared<ros_babel_fish::IntegratedDescriptionProvider>()) {
    //
}

BasicMessageMarshallingStageDescriptor::BasicMessageMarshallingStageDescriptor(const ros::NodeHandle& plugin_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider) 
: ProcessingStageDescriptor(plugin_namespace, "basic_marshaller"), m_message_description_provider(message_description_provider) {
    //
}


BasicMessageMarshallingStageDescriptor::~BasicMessageMarshallingStageDescriptor() { 
    //
}

    
bool BasicMessageMarshallingStageDescriptor::isModeSupported(const ProcessingStage::Mode mode) const  {
    return mode == ProcessingStage::Mode::FORWARD;
}

bool BasicMessageMarshallingStageDescriptor::isExclusiveToTopic(const ProcessingStage::Mode mode) const {
    return true;    
}

bool BasicMessageMarshallingStageDescriptor::isConcurrentlyExecutable(const ProcessingStage::Mode mode) const {
    return true;
}

    
const ros_babel_fish::DescriptionProvider::Ptr BasicMessageMarshallingStageDescriptor::getMessageDescriptionProvider() const {
    return m_message_description_provider;
}


ProcessingStage::Ptr BasicMessageMarshallingStageDescriptor::getStage(const ProcessingStage::Mode mode, const std::string& topic, const std::string& message_type) {
    switch(mode) {

        case ProcessingStage::Mode::FORWARD : {
            
            // Information on how the message is structured.
            const ros_babel_fish::MessageDescription::ConstPtr description = m_message_description_provider->getMessageDescription(message_type);

            return std::make_shared<BasicMessageSerializationStage>(description->message_template);
        }
        // TODO: We need a validate stage too.
        default: 
            throw std::invalid_argument("Cant handle requested mode.");
        
    }
}


}