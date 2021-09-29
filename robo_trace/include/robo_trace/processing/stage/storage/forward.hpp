#pragma once

// Std
#include <string>
#include <memory>
// Mongo
#include "robo_trace/config.h"
#include <mongo/client/dbclient.h>
// Project
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/stage/stage.hpp"


namespace robo_trace {

class StorageForwardStage final : public ProcessingStage { 

public:

    /**
     * 
     */
    StorageForwardStage(const DataContainer::Ptr& metadata, const std::shared_ptr<mongo::DBClientConnection>& connection, const std::string& database, const std::string& collection, const std::string& global_meta_store_path);
  
    /**
     * 
     */
    virtual ~StorageForwardStage();
    
    /**
     *
     */
    virtual ProcessingMode getMode() const final override;
    
    /**
     * 
     */
    virtual void process(const ProcessingContext::Ptr& context) final override;

private:

    /** */
    const std::string m_message_store_path;
    /** */
    const std::shared_ptr<mongo::DBClientConnection> m_connection;

};

}