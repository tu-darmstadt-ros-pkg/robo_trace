#pragma once
// Std
#include <string>
#include <vector>
#include <memory>
#include <regex>
// Project
#include "robo_trace_plugin_interface/processing/descriptor.hpp"


namespace robo_trace {

class PipelineDescriptor final {

public:

    typedef std::shared_ptr<PipelineDescriptor> Ptr;
    typedef std::shared_ptr<const PipelineDescriptor> ConstPtr;

public:

    /**
     * 
     */
    PipelineDescriptor(const std::string& name, const int priority, const std::regex regex, std::vector<ProcessingStageDescriptor::Ptr> descriptors);

    /**
     * 
     */
    ~PipelineDescriptor();

    /**
     * 
     */
    const std::string& getName() const;

    /**
     * 
     */
    const int getPriority() const;

    /**
     * 
     */
    const std::regex getRegex() const;

    /**
     * 
     */
    const std::vector<ProcessingStageDescriptor::Ptr> getDescriptors() const;

private:

    /** */
    const std::string m_name;
    
    /** */
    const int m_priority;
    /** */
    const std::regex m_regex;

    /** */
    const std::vector<ProcessingStageDescriptor::Ptr> m_descriptors;

};   

}