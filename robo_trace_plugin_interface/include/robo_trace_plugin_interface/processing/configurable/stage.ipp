// Base
#include "robo_trace_plugin_interface/processing/configurable/stage.hpp"


namespace robo_trace {

template <typename C>
ConfigurableProcessingStage::ConfigurableProcessingStage(const ProcessingStage::Mode mode, const std::string name, const std::shared_ptr<C> configuration) 
: ProcessingStage(mode, name), m_configuration(configuration) {
    //
}

template <typename C>
ConfigurableProcessingStage::~ConfigurableProcessingStage() = default;


template <typename C>
const std::shared_ptr<C>& ConfigurableProcessingStage::getConfiguration() {
    return m_configuration;
}

}