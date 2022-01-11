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

    /**
     * 
     */
    static void serialize(const ros_babel_fish::MessageTemplate::ConstPtr& msg_template, bsoncxx::builder::basic::sub_document& builder, const uint8_t* stream, size_t& bytes_read);

private:

    /** */
    ros_babel_fish::MessageTemplate::ConstPtr m_message_template;

};

}