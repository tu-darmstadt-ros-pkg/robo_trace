// Base
#include "robo_trace_plugin_interface/processing/stage.hpp"


namespace robo_trace {

ProcessingStage::ProcessingStage(const ProcessingStage::Mode mode, const std::string name) 
: m_mode(mode), m_name(name) {
    //
}

ProcessingStage::~ProcessingStage() = default;


const std::string& ProcessingStage::getName() const {
    return m_name;
}

const ProcessingStage::Mode ProcessingStage::getMode() const {
    return m_mode;
}

}