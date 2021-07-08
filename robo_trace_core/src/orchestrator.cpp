// Base
#include "robo_trace_core/orchestrator.hpp"
// Ros
#include <ros/console.h>
// Project
#include "robo_trace_core/utils/smart_ptr_conversions.hpp"
#include "robo_trace_core/definitions.hpp"
#include "robo_trace_core/processing/marshalling/descriptor.hpp"


namespace robo_trace {

RoboTraceOrchestrator::RoboTraceOrchestrator() 
: m_forward_pipeline_constructor(*this) {
    // 
};

RoboTraceOrchestrator::~RoboTraceOrchestrator() = default;


ros::NodeHandle& RoboTraceOrchestrator::getGloablNodeHandle() {
    return m_global_node_handle;
}

ros::NodeHandle& RoboTraceOrchestrator::getSystemNodeHandle() {
    return m_system_node_handle;
}


ros_babel_fish::BabelFish& RoboTraceOrchestrator::getBabelFish() {
    return m_fish;
}

MongoDBConnection::Ptr& RoboTraceOrchestrator::getDatabaseConnection() {
    return m_database_connection;
}

std::vector<RoboTraceProcessingPlugin::Ptr>& RoboTraceOrchestrator::getProcessingPlugins() {
    return m_processing_plugins;
}

std::vector<ProcessingStageDescriptor::Ptr>& RoboTraceOrchestrator::getProcessingStageDescriptors() {
    return m_processing_stage_descriptors;
}    

void RoboTraceOrchestrator::initialize(ros::NodeHandle &node_handle, ros::NodeHandle &node_handle_private) {
    
    /*
        Setup node handles.
    */

    m_system_node_handle = node_handle_private;
    m_global_node_handle = node_handle;

    ros::NodeHandle plugin_ns = ros::NodeHandle(m_system_node_handle, "plugins");
    ros::NodeHandle plugin_settings_ns = ros::NodeHandle(plugin_ns, "settings");
    
    /*
        Load in the database connection.
    */

    m_database_connection = std::make_shared<MongoDBConnection>("primary");
    m_database_connection->initialize(m_system_node_handle);

    /*
        Load in the basic marshaller.
    */

    ProcessingStageDescriptor::Ptr default_marshaller = std::make_shared<BasicMessageMarshallingStageDescriptor>(m_system_node_handle);
    m_processing_stage_descriptors.push_back(default_marshaller);

    /*
        Load the processing plugins.
    */

    m_processing_plugin_loader = std::make_unique<pluginlib::ClassLoader<RoboTraceProcessingPlugin>>("robo_trace_plugin_interface", "robo_trace::RoboTraceProcessingPlugin");

    std::vector<std::string> processing_plugin_names;
    plugin_ns.getParam("names", processing_plugin_names);

    for (const std::string& processing_plugin_name : processing_plugin_names) {
        
        // Load the plugin. 
        boost::shared_ptr<RoboTraceProcessingPlugin> plugin = m_processing_plugin_loader->createInstance(processing_plugin_name);
       
        try{
            // Setup. In the case of OpenSSL this for example will create keys.
            plugin->initialize(plugin_settings_ns);
        // Kind of bad practice. Though the plugin could throw anything at us.
        } catch (...) {
            std::exception_ptr exception = std::current_exception();
            ROS_INFO_STREAM("Failed initializing '" << plugin->getName() << 
                            "' plugin, due to '" << (exception ? exception.__cxa_exception_type()->name() : "unknown error") << "'.");
            // TODO: Probably rather just exit.
            continue;
        }
        
        RoboTraceProcessingPlugin::Ptr converted_ptr = to_std_ptr<RoboTraceProcessingPlugin>(plugin);
        m_processing_plugins.push_back(converted_ptr);
        
        for (const ProcessingStageDescriptor::Ptr& descriptor : converted_ptr->getDescriptors()) {
            m_processing_stage_descriptors.push_back(descriptor);
        }

    }

    /*
        Setup the forward pipeline constructor
    */

    m_forward_pipeline_constructor.initialize(m_global_node_handle, m_system_node_handle);

    /*
        Setup the update loop, checking for new topics to be recorded.
    */
    
    m_check_for_topics_timer = node_handle_private.createTimer(ros::Duration(0.5), &RoboTraceOrchestrator::onCheckForNewTopics, this);

    /*
    
    this->reconfigure = std::make_shared<ddynamic_reconfigure::DDynamicReconfigure>(node_handle_private);


    
    this->reconfigure->registerVariable<double>(DYNAMIC_RECONFIGURE_CHECK_FOR_TOPICS_PERIOD_NAME, DYNAMIC_RECONFIGURE_CHECK_FOR_TOPICS_PERIOD_DEFAULT, [this](double period) {
        
        ROS_DEBUG_STREAM(ROS_LOGGING_ORCHESTRATOR_NAME, "Check for new topic period is now " << period << "s.");
        
        if (period == 0.0) {
            this->timer_check_for_topics.stop();
        } else {

            this->timer_check_for_topics.setPeriod(ros::Duration(period));

            if (this->timer_check_for_topics_period == 0.0) {
                this->timer_check_for_topics.start();
            }
            
        }

        this->timer_check_for_topics_period = period;

    }, DYNAMIC_RECONFIGURE_CHECK_FOR_TOPICS_PERIOD_DESCRIPTION, DYNAMIC_RECONFIGURE_CHECK_FOR_TOPICS_PERIOD_MIN, DYNAMIC_RECONFIGURE_CHECK_FOR_TOPICS_PERIOD_MAX);
    */
    
}


void RoboTraceOrchestrator::onCheckForNewTopics(const ros::TimerEvent& event) {

    ros::master::V_TopicInfo topic_list;
    ros::master::getTopics(topic_list);

    for (ros::master::V_TopicInfo::iterator it = topic_list.begin() ; it != topic_list.end(); it++) {
        const ros::master::TopicInfo& info = *it;

        if (m_pipelines.find(info.name) != m_pipelines.end()) {
            ROS_INFO_STREAM("Topic " << info.name << " is already beeing recorded.");
            continue;
        }

        ForwardProcessingPipeline::Ptr pipeline = m_forward_pipeline_constructor.construct(ProcessingStage::Mode::FORWARD, info.name, info.datatype);
    
        std::pair<std::string, ForwardProcessingPipeline::Ptr> record(info.name, pipeline);
        m_pipelines.insert(record);


        std::cout << "Topic : " << it - topic_list.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
    
        pipeline->start();

    }

   
    // m_pipelines
}

}