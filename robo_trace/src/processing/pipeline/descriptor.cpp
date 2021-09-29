// Base
#include "robo_trace/processing/pipeline/descriptor.hpp"


namespace robo_trace {

PipelineDescriptor::PipelineDescriptor(const std::string& name, const int priority, const std::regex regex, std::vector<ProcessingStageDescriptor::Ptr> descriptors) 
: m_name(name), m_priority(priority), m_regex(regex), m_descriptors(descriptors) {
    // 
}

PipelineDescriptor::~PipelineDescriptor() = default;

const std::string& PipelineDescriptor::getName() const {
    return m_name;
}

const int PipelineDescriptor::getPriority() const {
    return m_priority;
}

const std::regex PipelineDescriptor::getRegex() const {
    return m_regex;
}

const std::vector<ProcessingStageDescriptor::Ptr> PipelineDescriptor::getDescriptors() const {
    return m_descriptors;
}

}