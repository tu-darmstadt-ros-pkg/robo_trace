// Base
#include "robo_trace_core/persistance/store.hpp"
// Ros
#include "ros/console.h"
// Mongo
#include "robo_trace_core/config.h"
#include <mongo/bson/bsonobj.h>
#include <mongo/bson/bsonobjbuilder.h>
// Project
#include "robo_trace_core/persistance/connection.hpp"
#include "robo_trace_core/persistance/bson_backed_lazy_metadata_container.hpp"


namespace robo_trace {

MessageStore::MessageStore(const MongoDBConnection& connection, const std::string& database, const std::string& collection) 
: m_database(database), m_collection(collection), m_store_path(database + "." + collection), m_connection(connection) {
    // Setup GFS
    //m_grid_fs = std::make_shared<mongo::GridFS>(*m_plugin.getConnection(), m_database);

    // const std::string meta_collection = m_database + ".ros_message_collections";

    // Do we need to store the message schema?
    // m_store_schema = !m_plugin.getConnection()->count(meta_collection, BSON("name" << m_collection));
  
}
   
MessageStore::~MessageStore() {

}

   
const std::string& MessageStore::getDatabaseName() const {
    return m_database;
}

const std::string& MessageStore::getCollectionName() const {
    return m_collection;
}

const std::string& MessageStore::getStorePath() const {
    return m_store_path;
}


MetadataContainer::Ptr MessageStore::getContainer() {
    return std::make_shared<LazyMongoBsonMetadataContainer>();
}

void MessageStore::store(const MessageProcessingContext::Ptr& context) {
    /*
    if (m_store_schema) {
        ROS_INFO_STREAM(" - Storing schema");
        const std::string meta_collection = m_database + ".ros_message_collections";
        m_plugin.getConnection()->insert(meta_collection, BSON("name" << m_collection << "type" << context->getMessage()->getDescription()->dataType() << "md5sum" << context->getMessage()->getDescription()->md5Sum()));

        m_plugin.getConnection()->createIndex(getStoreName(), BSON("creation_time" << 1));

        m_store_schema = false;
    } */


    
    // Constructor for the message record.
    mongo::BSONObjBuilder builder;
    builder.genOID();
    
    /*
        Serialize the metadata.
    */

    MetadataContainer::Ptr abstract_metadata = context->getMetadata();
    // I feel bad for using a dynamic cast here.
    std::shared_ptr<LazyMongoBsonMetadataContainer> mongo_metadata = std::dynamic_pointer_cast<LazyMongoBsonMetadataContainer>(abstract_metadata);

    
    mongo::BufBuilder& metadata_buf_builder = builder.subobjStart("metadata");
    mongo::BSONObjBuilder metadata_builder(metadata_buf_builder);

    mongo_metadata->serialize(metadata_builder);
  
    metadata_builder.done();
    

    /*
        Serialize the message.
    */


    const mongo::BSONObj& message = context->getMessage()->getSerialized();
    builder.append("message", message);

    /*
        Push the message to MongoDB.
    */

    mongo::BSONObj entry = builder.obj();
    m_connection.getConnection()->insert(m_store_path, entry);

}


void MessageStore::store(const MetadataContainer::Ptr& metadata) {
    // TODO: Maybe used for dumping arbitrary information
}


}