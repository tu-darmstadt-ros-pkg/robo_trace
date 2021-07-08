#pragma once

// Std
#include <memory>
// Project
#include "robo_trace_plugin_interface/processing/stage.hpp"


namespace robo_trace {

template<class C>
class ConfigurableProcessingStage : public ProcessingStage {

public:


    /**
     * 
     */
    ConfigurableProcessingStage(const ProcessingStage::Type type, const std::string name, const std::shared_ptr<C> configuration);

    /**
     * 
     */
    virtual ~ConfigurableProcessingStage();

    /**
     * Provides a pointer to the configuration object associated to this
     * processing stage.
     * 
     * @returns a pointer to the configuration object.
     */
    const std::shared_ptr<C>& getConfiguration();

protected:

    const std::shared_ptr<C> m_configuration;

};

}