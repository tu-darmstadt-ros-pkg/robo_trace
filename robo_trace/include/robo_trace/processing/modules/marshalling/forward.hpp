#pragma once

// MongoCXX
#include <bsoncxx/builder/basic/document.hpp>
// BabelFish
#include <ros_babel_fish/generation/message_template.h>
#include <ros_babel_fish/generation/description_provider.h> 
// Project
#include "robo_trace/processing/processor.hpp"


namespace robo_trace::processing {

class BasicMarshallingForwardProcessor final : public Processor { 

public:

    /**
     * 
     */
    BasicMarshallingForwardProcessor(const ros_babel_fish::MessageTemplate::ConstPtr message_template);
  
    /**
     * 
     */
    virtual ~BasicMarshallingForwardProcessor();
    
    /**
     *
     */
    virtual Mode getMode() const final override;

    /**
     * 
     */
    virtual void process(const Context::Ptr& context) final override;

private:

    /** */
    ros_babel_fish::MessageTemplate::ConstPtr m_message_template;

};

}