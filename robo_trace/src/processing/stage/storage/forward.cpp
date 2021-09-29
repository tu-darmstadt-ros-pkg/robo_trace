// Project
#include "robo_trace/processing/stage/storage/forward.hpp"
// Std
#include <mutex>


namespace robo_trace {


StorageForwardStage::StorageForwardStage(const DataContainer::Ptr& metadata, const std::shared_ptr<mongo::DBClientConnection>& connection, const std::string& database, const std::string& collection, const std::string& global_meta_store_path) 
: m_message_store_path(database + "." + collection), 
  m_connection(connection) {

    // Constructor for the data record.
    mongo::BSONObjBuilder builder;
    builder.append("collection", collection);
    builder.genOID();

    /*
        Serialize the metadata.
    */

    metadata->serialize(builder);
  
    /*
        Push the message to MongoDB.
    */
    
    // The connection is unique to this stage (, which is not executed concurrently!). 
    m_connection->insert(global_meta_store_path, builder.obj());
   
}

StorageForwardStage::~StorageForwardStage() {
    //
}

ProcessingMode StorageForwardStage::getMode() const {
    return ProcessingMode::CAPTURE;
}

void StorageForwardStage::process(const ProcessingContext::Ptr& context) {

    // Constructor for the message record.
    mongo::BSONObjBuilder builder;
    builder.append("type", 1);
    builder.genOID();
    
    /*
        Serialize the metadata.
    */

     
    mongo::BufBuilder& metadata_buf_builder = builder.subobjStart("metadata");
    mongo::BSONObjBuilder metadata_builder(metadata_buf_builder);

    context->getMetadata()->serialize(metadata_builder);
  
    metadata_builder.done();

    /*
        Serialize the message.
    */

    
    if (!context->isSerialized()) {
        throw std::runtime_error("Message has reached database writeback stage, but is not serialized yet.");
    }

    const mongo::BSONObj& serialized_message = context->getSerializedMessage().value();

    builder.append("message", serialized_message);

    /*
        Push the message to MongoDB.
    */
    
    m_connection->insert(m_message_store_path, builder.obj());

}



}