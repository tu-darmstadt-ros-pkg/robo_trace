#pragma once

// MongoCXX
#include <bsoncxx/document/view.hpp>
// BabelFish
#include <ros_babel_fish/message_description.h>
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
#include <ros_babel_fish/messages/compound_message.h>
// Project
#include "robo_trace/processing/processor.hpp"


namespace robo_trace::processing {

class BasicMarshallingBackwardProcessor final : public Processor { 

public:

    /**
     * 
     */
    BasicMarshallingBackwardProcessor(const ros_babel_fish::MessageDescription::ConstPtr& message_description);
  
    /**
     * 
     */
    virtual ~BasicMarshallingBackwardProcessor();

    /**
     *
     */
    virtual Mode getMode() const final override;
   
    /**
     * 
     */
    virtual void process(const Context::Ptr& context) final override;

private:

    /**
     * 
     */
    void deserialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, const bsoncxx::document::view& serialized, ros_babel_fish::CompoundMessage& deserialized);

private:

    /** */
    ros_babel_fish::MessageDescription::ConstPtr m_message_description;

};

}