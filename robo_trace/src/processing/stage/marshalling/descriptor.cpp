// Base
#include "robo_trace/processing/stage/marshalling/descriptor.hpp"
// Std
#include <stdexcept>
// BabelFish
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
// Project
#include "robo_trace/processing/stage/marshalling/forward.hpp"
#include "robo_trace/processing/stage/marshalling/backward.hpp"


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
  
bool BasicMessageMarshallingStageDescriptor::isModeSupported(const ProcessingMode mode) const  {
    return mode == ProcessingMode::CAPTURE || mode == ProcessingMode::REPLAY;
}

std::optional<ProcessingStage::Ptr> BasicMessageMarshallingStageDescriptor::getStage(const DataContainer::Ptr& summary, const ProcessingMode mode) {

    const std::string message_type = summary->getString("message_type");
    const ros_babel_fish::MessageDescription::ConstPtr message_description = m_message_description_provider->getMessageDescription(message_type); 

    if (message_description == nullptr) {
        throw std::runtime_error("Failed finding message description.");
    }

    switch(mode) {

        case ProcessingMode::CAPTURE: 
            return std::make_shared<BasicMessageMarshallingForwardStage>(message_description->message_template);
        
        case ProcessingMode::REPLAY:
            return std::make_shared<BasicMessageMarshallingBackwardStage>(message_description);

        default: 
            throw std::runtime_error("Cant handle requested mode.");
        
    }
}


}