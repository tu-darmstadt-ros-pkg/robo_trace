// Base
#include "robo_trace/processing/modules/marshalling/descriptor.hpp"
// Std
#include <stdexcept>
// BabelFish
#include <ros_babel_fish/generation/providers/integrated_description_provider.h>
// Project
#include "robo_trace/processing/modules/marshalling/forward.hpp"
#include "robo_trace/processing/modules/marshalling/backward.hpp"


namespace robo_trace::processing {

BasicMarshallingModuleDescriptor::BasicMarshallingModuleDescriptor(const ros::NodeHandle& plugin_namespace) 
: BasicMarshallingModuleDescriptor(plugin_namespace, std::make_shared<ros_babel_fish::IntegratedDescriptionProvider>()) {
    //
}

BasicMarshallingModuleDescriptor::BasicMarshallingModuleDescriptor(const ros::NodeHandle& plugin_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider) 
: Descriptor(plugin_namespace, "basic_marshaller"), m_message_description_provider(message_description_provider) {
    //
}

BasicMarshallingModuleDescriptor::~BasicMarshallingModuleDescriptor() { 
    //
}
  
bool BasicMarshallingModuleDescriptor::isModeSupported(const Mode mode) const  {
    return mode == Mode::CAPTURE || mode == Mode::REPLAY;
}

std::optional<Processor::Ptr> BasicMarshallingModuleDescriptor::getStage(const robo_trace::store::Container::Ptr& summary, const Mode mode) {

    const std::string message_type = summary->getString("message_type");
    const ros_babel_fish::MessageDescription::ConstPtr message_description = m_message_description_provider->getMessageDescription(message_type); 

    if (message_description == nullptr) {
        throw std::runtime_error("Failed finding message description.");
    }

    switch(mode) {

        case Mode::CAPTURE: 
            return std::make_shared<BasicMarshallingForwardProcessor>(message_description->message_template);
        
        case Mode::REPLAY:
            return std::make_shared<BasicMarshallingBackwardProcessor>(message_description);

        default: 
            throw std::runtime_error("Cant handle requested mode.");
        
    }
}


}