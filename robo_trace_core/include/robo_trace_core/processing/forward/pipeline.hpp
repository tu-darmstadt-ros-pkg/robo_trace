#pragma once

// Std
#include <memory>
// Ros
#include <ros/ros.h>
// Babel Fish
#include <ros_babel_fish/babel_fish.h>
// Project
#include "robo_trace_plugin_interface/processing/stage.hpp"
#include "robo_trace_core/persistance/store.hpp"
#include "robo_trace_core/processing/invocation/invoker.hpp"


namespace robo_trace {

class ForwardProcessingPipeline {

public:

    typedef std::shared_ptr<ForwardProcessingPipeline> Ptr;
    typedef std::shared_ptr<const ForwardProcessingPipeline> ConstPtr;

public:

    /**
     * 
     */
    ForwardProcessingPipeline(ros_babel_fish::BabelFish& fish, ros::NodeHandle& node_handle, const MessageStore::Ptr& store, const std::string& topic);

    /**
     * 
     */
    ~ForwardProcessingPipeline();

    /**
     * 
     */
    ros::NodeHandle& getNodeHandle();

    /**
     * 
     */
    ros_babel_fish::BabelFish& getBabelFish();

    /**
     * 
     */
    const std::string& getTopic() const;
    
    /**
     * 
     */
    const MessageStore::Ptr& getStore() const;

    /**
     * 
     */
    std::vector<ProcessingStage::Ptr>& getStages();

    /**
     * 
     */
    const ProcessingStageInvoker::Ptr& getInvoker() const;

    /**
     * 
     */
    void setInvoker(const ProcessingStageInvoker::Ptr& invoker);

    /**
     * 
     */
    void start();

    /**
     * 
     */
    void stop();

    /**
     * 
     * 
     * TODO: Is it feasible to retrieve a non const pointer here? From
     *   the API docs there seems to be such method signature present.
     */
    void process(const ros_babel_fish::BabelFishMessage::ConstPtr& msg);

private:   

    /** */
    ros::NodeHandle& m_node_handle;
    /** */
    ros_babel_fish::BabelFish& m_fish;

    /** */
    const std::string m_topic;
    /** */
    bool m_subscriber_active = false;
    /** */
    ros::Subscriber m_subscriber;

    /** */
    const MessageStore::Ptr m_store;

    /** */
    ProcessingStageInvoker::Ptr m_invoker; 
    /** */
    std::vector<ProcessingStage::Ptr> m_stages;
    
};

}