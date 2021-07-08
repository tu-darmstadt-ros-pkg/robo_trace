// Base
#include "robo_trace_plugin_interface/processing/descriptor.hpp"


namespace robo_trace {

ProcessingStageDescriptor::ProcessingStageDescriptor(const ros::NodeHandle& plugin_namespace, const std::string name) 
: m_name(name){
    
    // Namespace is nested one level down under plugin general ns.
    m_handle = ros::NodeHandle(plugin_namespace, name);

}

ProcessingStageDescriptor::~ProcessingStageDescriptor() = default;


const std::string& ProcessingStageDescriptor::getName() const {
    return m_name;
}

ros::NodeHandle& ProcessingStageDescriptor::getHandle() {
    return m_handle;
}





}