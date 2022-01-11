// Base
#include "robo_trace/processing/variant.hpp"


namespace robo_trace::processing {

ProcessingVariant::ProcessingVariant(const std::string& name, const int priority, const std::regex regex, std::vector<Descriptor::Ptr> descriptors) 
: m_name(name), m_priority(priority), m_regex(regex), m_descriptors(descriptors) {
    // 
}

ProcessingVariant::~ProcessingVariant() = default;

const std::string& ProcessingVariant::getName() const {
    return m_name;
}

const int ProcessingVariant::getPriority() const {
    return m_priority;
}

const std::regex ProcessingVariant::getRegex() const {
    return m_regex;
}

const std::vector<Descriptor::Ptr> ProcessingVariant::getDescriptors() const {
    return m_descriptors;
}

}