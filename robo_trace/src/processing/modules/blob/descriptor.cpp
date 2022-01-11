// Base
#include "robo_trace/processing/stage/blob/descriptor.hpp"
// Std
#include <stdexcept>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/stage/blob/forward.hpp"
#include "robo_trace/processing/stage/blob/backward.hpp"


namespace robo_trace {

BlobDecouplingStageDescriptor::BlobDecouplingStageDescriptor(const ConnectionOptions::ConstPtr& options, const ros::NodeHandle& stages_namespace) 
: ProcessingStageDescriptor(stages_namespace, "blob_decoupling"), 
  m_connector_options(options) {
      //
}

BlobDecouplingStageDescriptor::~BlobDecouplingStageDescriptor() = default;
    
bool BlobDecouplingStageDescriptor::isModeSupported(const ProcessingMode mode) const  {
    return mode == ProcessingMode::CAPTURE || mode == ProcessingMode::REPLAY;
}

std::optional<ProcessingStage::Ptr> BlobDecouplingStageDescriptor::getStage(const DataContainer::Ptr& summary, const ProcessingMode mode) {
    
    // Attempt to establish a connection with the database. Could block a little.
    const std::shared_ptr<mongo::DBClientConnection> connection = ConnectionProvider::getConnection(m_connector_options); 
            
    switch(mode) {

        case ProcessingMode::CAPTURE : {
            return std::make_shared<BlobDecouplingForwardStage>(
                // connection
                connection, 
                // database
                m_connector_options->m_database_name
            );    
        }
        case ProcessingMode::REPLAY: {
            return std::make_shared<BlobDecouplingBackwardStage>(
                // connection
                connection, 
                // database
                m_connector_options->m_database_name
            );    
        }

        default: 
            throw std::runtime_error("Cant serve the requested mode.");
        
    }
}


}