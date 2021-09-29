// Base
#include "robo_trace/processing/stage/storage/descriptor.hpp"
// Std
#include <stdexcept>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/stage/storage/forward.hpp"


namespace robo_trace {

StorageStageDescriptor::StorageStageDescriptor(const ConnectionOptions::ConstPtr& options, const ros::NodeHandle& plugin_namespace) 
: ProcessingStageDescriptor(plugin_namespace, "storage"), 
  m_summary_collection_path(options->m_database_name + "." + options->m_summary_collection_name),
  m_connector_options(options) {
      //
}

StorageStageDescriptor::~StorageStageDescriptor() = default;
    
bool StorageStageDescriptor::isModeSupported(const ProcessingMode mode) const  {
    return mode == ProcessingMode::CAPTURE;
}

std::optional<ProcessingStage::Ptr> StorageStageDescriptor::getStage(const DataContainer::Ptr& chain_metadata, const ProcessingMode mode) {
    switch(mode) {

        case ProcessingMode::CAPTURE : {

            // Attempt to establish a connection with the database. Could block a little.
            const std::shared_ptr<mongo::DBClientConnection> connection = ConnectionProvider::getConnection(m_connector_options); 
            // TODO: Defines for common metadata fields.
            const std::string collection_name = chain_metadata->getString("topic");
            
            return std::make_shared<StorageForwardStage>(
                // metadata
                chain_metadata, 
                // connection
                connection, 
                // database
                m_connector_options->m_database_name, 
                // collection
                collection_name, 
                // global_meta_store_path
                m_summary_collection_path
            );
        }

        default: 
            throw std::runtime_error("Cant serve the requested mode.");
        
    }
}


}