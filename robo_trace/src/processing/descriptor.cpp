// Base
#include "robo_trace/processing/descriptor.hpp"


namespace robo_trace::processing {

Descriptor::Descriptor(const ros::NodeHandle& stage_namespace, const std::string& name) 
: m_name(name){
    
    // Namespace is nested one level down under plugin general ns.
    m_handle = ros::NodeHandle(stage_namespace, name);

}

Descriptor::~Descriptor() = default;


const std::string& Descriptor::getName() const {
    return m_name;
}

ros::NodeHandle& Descriptor::getNodeHandle() {
    return m_handle;
}

}