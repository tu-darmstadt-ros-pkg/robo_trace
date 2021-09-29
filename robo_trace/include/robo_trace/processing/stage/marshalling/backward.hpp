#pragma once

// BabelFish
#include <ros_babel_fish/message_description.h>
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
#include <ros_babel_fish/messages/compound_message.h>
// Project
#include "robo_trace/processing/stage/stage.hpp"


namespace robo_trace {

class BasicMessageMarshallingBackwardStage final : public ProcessingStage { 

public:

    /**
     * 
     */
    BasicMessageMarshallingBackwardStage(const ros_babel_fish::MessageDescription::ConstPtr& message_description);
  
    /**
     * 
     */
    virtual ~BasicMessageMarshallingBackwardStage();

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
    void deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const mongo::BSONObj& serialized, ros_babel_fish::CompoundMessage& deserialized);

private:

    /** */
    ros_babel_fish::MessageDescription::ConstPtr m_message_description;

};

}