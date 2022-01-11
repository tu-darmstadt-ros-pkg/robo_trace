// Base
#include "robo_trace/processing/modules/storage/descriptor.hpp"
// Std
#include <stdexcept>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/modules/storage/forward.hpp"


namespace robo_trace::processing {

StorageModuleDescriptor::StorageModuleDescriptor(const robo_trace::store::Options::ConstPtr& options, const ros::NodeHandle& plugin_namespace) 
: Descriptor(plugin_namespace, "storage"), 
  m_connector_options(options) {
      //
}

StorageModuleDescriptor::~StorageModuleDescriptor() = default;
    
bool StorageModuleDescriptor::isModeSupported(const Mode mode) const  {
    return mode == Mode::CAPTURE;
}

std::optional<Processor::Ptr> StorageModuleDescriptor::getStage(const robo_trace::store::Container::Ptr& chain_metadata, const Mode mode) {
    switch(mode) {

        case Mode::CAPTURE : {

           // TODO: Defines for common metadata fields.
            const std::string collection_name = chain_metadata->getString("topic");
            
            return std::make_shared<StorageForwardProcessor>(
                // metadata
                chain_metadata, 
                // database
                m_connector_options->m_database_name, 
                // collection_name_data
                collection_name, 
                // collection_name_meta
                m_connector_options->m_collection_name_summary
            );
        }

        default: 
            throw std::runtime_error("Cant serve the requested mode.");
        
    }
}


}