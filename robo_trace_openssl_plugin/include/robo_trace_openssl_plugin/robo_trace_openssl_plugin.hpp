#pragma once
// Std
#include <vector>
#include <memory>
// Project
#include "robo_trace/processing/plugin.hpp"
#include "robo_trace/processing/descriptor.hpp"
#include "robo_trace_openssl_plugin/key_manager.hpp"


namespace robo_trace::plugin::open_ssl {
    
class RoboTraceOpenSSLPlugin final : public robo_trace::processing::Plugin {

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
    const KeyManager::Ptr& getKeyManager() const;

    /**
     * 
     */
    virtual std::vector<robo_trace::processing::Descriptor::Ptr> setup() final override;

private:

    /** */
    KeyManager::Ptr m_key_manager;

};

} 
