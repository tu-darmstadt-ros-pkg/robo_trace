#pragma once
// Ros
#include <ros/ros.h>
// BabelFish
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace_plugin_interface/processing/descriptor.hpp"


namespace robo_trace {

class BasicMessageMarshallingStageDescriptor final : public ProcessingStageDescriptor {

public:

    /**
     * 
     */
    BasicMessageMarshallingStageDescriptor(const ros::NodeHandle& plugin_namespace);

    /**
     * 
     */
    BasicMessageMarshallingStageDescriptor(const ros::NodeHandle& plugin_namespace, const ros_babel_fish::DescriptionProvider::Ptr message_description_provider);

    /**
     * 
     */
    virtual ~BasicMessageMarshallingStageDescriptor();

    /**
     * 
     */
    virtual bool isModeSupported(const ProcessingStage::Mode mode) const final override;
   
    /**
     * 
     */
    virtual bool isExclusiveToTopic(const ProcessingStage::Mode mode) const final override;

    /**
     * 
     */
    virtual bool isConcurrentlyExecutable(const ProcessingStage::Mode mode) const final override; 
    
    /**
     * 
     */
    const ros_babel_fish::DescriptionProvider::Ptr getMessageDescriptionProvider() const;

    /**
     * 
     */
    virtual ProcessingStage::Ptr getStage(const ProcessingStage::Mode mode, const std::string& topic, const std::string& message_type) final override;

private:

    /** */
    const ros_babel_fish::DescriptionProvider::Ptr m_message_description_provider;

};

}