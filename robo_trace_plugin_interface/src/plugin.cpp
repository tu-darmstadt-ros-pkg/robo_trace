// Base
#include "robo_trace_plugin_interface/plugin.hpp"


namespace robo_trace {

RoboTraceProcessingPlugin::RoboTraceProcessingPlugin(const std::string& name) 
: m_name(name) {
    //
}

RoboTraceProcessingPlugin::~RoboTraceProcessingPlugin() = default;


const std::string& RoboTraceProcessingPlugin::getName() const {
    return m_name;
}

const ros::NodeHandle& RoboTraceProcessingPlugin::getSystemNodeHandle() const {
    return m_system_node_handle;
}

const ros::NodeHandle& RoboTraceProcessingPlugin::getPluginNodeHandle() const {
    return m_plugin_node_handle;
}


void RoboTraceProcessingPlugin::initialize(ros::NodeHandle& system_node_handle) {

    /*
        Setup node handles.
    */

    m_system_node_handle = system_node_handle;
    m_plugin_node_handle = ros::NodeHandle(m_system_node_handle, m_name);

    /*
        Setup the plugin itself.
    */

    setup();

}

void RoboTraceProcessingPlugin::setup() {
    // 
}

}