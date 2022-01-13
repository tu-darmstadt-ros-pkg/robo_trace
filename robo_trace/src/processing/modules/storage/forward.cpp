// Base
#include "robo_trace/processing/modules/storage/forward.hpp"
// Std
#include <string>
#include <mutex>
#include <stdexcept>
// MongoCXX
#include <bsoncxx/json.hpp>
#include <bsoncxx/stdx/optional.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
// Project
#include "robo_trace/parameters.hpp"


namespace robo_trace::processing {

StorageForwardProcessor::StorageForwardProcessor(const robo_trace::store::Container::Ptr& metadata, const std::string& database_name, const std::string& collection_name_data, const std::string& collection_name_meta) 
: m_database_name(database_name), m_collection_name_data(collection_name_data) {

    /*
        Retrieve a connection to the database and setup.
    */

    auto client = robo_trace::store::Connector::instance().getClient();
    auto database = (*client)[database_name];
    
    if (!database.has_collection(collection_name_meta)) {
        database.create_collection(collection_name_meta);
    }

    /*
        Serialize and persist the metadata.
    */

    bsoncxx::builder::basic::document builder{};
    builder.append(bsoncxx::builder::basic::kvp("collection", collection_name_data));
    
    metadata->serialize(builder);
 
    // The connection is unique to this stage (, which is not executed concurrently!). 
    bsoncxx::stdx::optional<mongocxx::result::insert_one > result = database[collection_name_meta].insert_one(builder.view());

    if (!result || result.value().result().inserted_count() != 1) {
        throw std::runtime_error("Failed storing metadata for collection " + collection_name_data + "!");
    }

    /*
        Create and organize the collection for data storage.
    */
    
    if (!database.has_collection(collection_name_data)) {
        database.create_collection(collection_name_data);
    } else {
#ifdef PROCESSING_MODULE_BASIC_STORE_FAIL_IF_DATA_COLLECTION_PRESENT      
        throw std::runtime_error("Data collection already present for " + collection_name_data + "!");
#endif
    }

    mongocxx::collection collection_data = database[collection_name_data];

    mongocxx::options::index collection_data_index_options{};
#ifdef PROCESSING_MODULE_BASIC_STORE_TREAT_TIME_AS_UNIQUE    
    collection_data_index_options.unique(true);
#endif

    collection_data.create_index(
        // keys
        bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp("metadata.time", 1)),
        // index_options
        collection_data_index_options
    );

}

StorageForwardProcessor::~StorageForwardProcessor() {
    //
}

Mode StorageForwardProcessor::getMode() const {
    return Mode::CAPTURE;
}

void StorageForwardProcessor::process(const Context::Ptr& context) {

    bsoncxx::builder::basic::document builder{};
    
    /*
        Serialize the metadata.
    */

    builder.append(bsoncxx::builder::basic::kvp("metadata", 
        [&context](bsoncxx::builder::basic::sub_document sub_document_builder) {
            context->getMetadata()->serialize(sub_document_builder);
        }));
   

    /*
        Serialize the message.
    */
    
    const std::optional<bsoncxx::document::view>& serialized_message_o = context->getBsonMessage();

    if (!serialized_message_o) {
        throw std::runtime_error("Failed persisting message! Writeback stage reached, but not yet serialized.");
    }

    builder.append(bsoncxx::builder::basic::kvp("message", serialized_message_o.value()));

    /*
        Push the message to MongoDB.
    */

    auto client = robo_trace::store::Connector::instance().getClient();
    bsoncxx::stdx::optional<mongocxx::result::insert_one> result = (*client)[m_database_name][m_collection_name_data].insert_one(builder.view());

#ifdef PROCESSING_MODULE_BASIC_STORE_VALIDATE_MESSAGE_WRITEBACK
    if (!result || result.value().result().inserted_count() != 1) {
        throw std::runtime_error("Failed storing message for collection " + m_collection_name_data + "!");
    }
#endif

}



}