// Base
#include "robo_trace/processing/plugin/plugin.hpp"


namespace robo_trace {

RoboTraceProcessingPlugin::RoboTraceProcessingPlugin(const std::string& name) 
: m_name(name) {
    //
}

RoboTraceProcessingPlugin::~RoboTraceProcessingPlugin() = default;


const std::string& RoboTraceProcessingPlugin::getName() const {
    return m_name;
}

const std::vector<ProcessingStageDescriptor::Ptr>& RoboTraceProcessingPlugin::getDescriptors() const {
    return m_descriptors;
}

ros::NodeHandle& RoboTraceProcessingPlugin::getSystemNodeHandle() {
    return m_system_node_handle;
}

ros::NodeHandle& RoboTraceProcessingPlugin::getPluginNodeHandle() {
    return m_plugin_node_handle;
}

ros::NodeHandle& RoboTraceProcessingPlugin::getStageNodeHandle() {
    return m_stage_node_handle;
}

void RoboTraceProcessingPlugin::initialize(ros::NodeHandle& system_node_handle, ros::NodeHandle& plugin_node_handle, ros::NodeHandle& stage_node_handle) {

    /*
        Setup node handles.
    */

    m_system_node_handle = system_node_handle;
    m_plugin_node_handle = ros::NodeHandle(plugin_node_handle, m_name);
    m_stage_node_handle = stage_node_handle;

    /*
        Setup the plugin itself.
    */

    m_descriptors = setup();

}

}