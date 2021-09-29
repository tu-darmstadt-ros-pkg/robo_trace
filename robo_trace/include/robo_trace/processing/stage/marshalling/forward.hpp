#pragma once

// BabelFish
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/processing/stage/stage.hpp"


namespace robo_trace {

class BasicMessageMarshallingForwardStage final : public ProcessingStage { 

public:

    /**
     * 
     */
    BasicMessageMarshallingForwardStage(const ros_babel_fish::MessageTemplate::ConstPtr message_template);
  
    /**
     * 
     */
    virtual ~BasicMessageMarshallingForwardStage();
    
    /**
     *
     */
    virtual ProcessingMode getMode() const final override;

    /**
     * 
     */
    virtual void process(const ProcessingContext::Ptr& context) final override;

private:

    /**
     * 
     */
    void serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, mongo::BSONObjBuilder& builder, const uint8_t* stream, size_t& bytes_read);

private:

    /** */
    ros_babel_fish::MessageTemplate::ConstPtr m_message_template;

};

}