#pragma once
// Std
#include <vector>
#include <memory>
// Project
#include "robo_trace_plugin_interface/plugin.hpp"
#include "robo_trace_openssl_plugins/key_manager.hpp"


namespace robo_trace {
    
class RoboTraceOpenSSLPlugins final : public RoboTraceProcessingPlugin {

public:

    /**
     * 
     */
    RoboTraceOpenSSLPlugins();

    /**
     * 
     */
    virtual ~RoboTraceOpenSSLPlugins();

    /**
     * 
     */
    const OpenSSLPluginKeyManager::Ptr& getKeyManager() const;

    /**
     * 
     */
    virtual const std::vector<ProcessingStageDescriptor::Ptr>& getDescriptors() const final override;

    /**
     * 
     */
    virtual void setup() final override;

private:

    /** */
    OpenSSLPluginKeyManager::Ptr m_key_manager;

    /** */
    std::vector<ProcessingStageDescriptor::Ptr> m_descriptors;

};

} 
