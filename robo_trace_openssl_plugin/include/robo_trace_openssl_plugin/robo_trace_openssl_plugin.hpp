#pragma once
// Std
#include <vector>
#include <memory>
// Project
#include "robo_trace/processing/plugin/plugin.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"


namespace robo_trace {
    
class RoboTraceOpenSSLPlugin final : public RoboTraceProcessingPlugin {

public:

    /**
     * 
     */
    RoboTraceOpenSSLPlugin();

    /**
     * 
     */
    virtual ~RoboTraceOpenSSLPlugin();

    /**
     * 
     */
    const OpenSSLPluginKeyManager::Ptr& getKeyManager() const;

    /**
     * 
     */
    virtual std::vector<ProcessingStageDescriptor::Ptr> setup() final override;

private:

    /** */
    OpenSSLPluginKeyManager::Ptr m_key_manager;

};

} 
