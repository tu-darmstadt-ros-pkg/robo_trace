#pragma once
// Project
#include "robo_trace_plugin_interface/processing/stage.hpp"
// BabelFish
#include <ros_babel_fish/generation/message_template.h>


namespace robo_trace {

class BasicMessageSerializationStage final : public ProcessingStage { 

public:

    /**
     * 
     */
    BasicMessageSerializationStage(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template);
  
    /**
     * 
     */
    virtual ~BasicMessageSerializationStage();

    /**
     * 
     */
    const ros_babel_fish::MessageTemplate::ConstPtr& getMessageTemplate() const;

    /**
     * 
     */
    virtual void process(MessageProcessingContext::Ptr& context) final override;

private:

    /**
     * 
     */
    void serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, mongo::BSONObjBuilder& builder, const uint8_t* stream, size_t& bytes_read);

private:

    /** */
    const ros_babel_fish::MessageTemplate::ConstPtr m_msg_template;

};

}