#pragma once

// Std
#include <string>
#include <memory>
// Project
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/processor.hpp"


namespace robo_trace::processing {

class StorageForwardProcessor final : public Processor { 

public:

    /**
     * 
     */
    StorageForwardProcessor(const robo_trace::store::Container::Ptr& metadata, const std::string& database, const std::string& collection_name_data, const std::string& collection_name_meta);
  
    /**
     * 
     */
    virtual ~StorageForwardProcessor();
    
    /**
     *
     */
    virtual Mode getMode() const final override;
    
    /**
     * 
     */
    virtual void process(const Context::Ptr& context) final override;

private:

    /** */
    const std::string m_database_name;
    /** */
    const std::string m_collection_name_data;

};

}