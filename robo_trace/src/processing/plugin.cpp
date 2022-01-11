// Base
#include "robo_trace/processing/plugin.hpp"


namespace robo_trace::processing {

Plugin::Plugin(const std::string& name) 
: m_name(name) {
    //
}

Plugin::~Plugin() = default;


const std::string& Plugin::getName() const {
    return m_name;
}

const std::vector<Descriptor::Ptr>& Plugin::getModules() const {
    return m_modules;
}

ros::NodeHandle& Plugin::getSystemNodeHandle() {
    return m_system_node_handle;
}

ros::NodeHandle& Plugin::getPluginNodeHandle() {
    return m_plugin_node_handle;
}

ros::NodeHandle& Plugin::getStageNodeHandle() {
    return m_stage_node_handle;
}

void Plugin::initialize(ros::NodeHandle& system_node_handle, ros::NodeHandle& plugin_node_handle, ros::NodeHandle& stage_node_handle) {

    /*
        Setup node handles.
    */

    m_system_node_handle = system_node_handle;
    m_plugin_node_handle = ros::NodeHandle(plugin_node_handle, m_name);
    m_stage_node_handle = stage_node_handle;

    /*
        Setup the plugin itself.
    */

    m_modules = setup();

}

}