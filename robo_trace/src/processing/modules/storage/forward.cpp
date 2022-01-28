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
// Project
#include "robo_trace/parameters.hpp"


namespace robo_trace::processing {

StorageForwardProcessor::StorageForwardProcessor(const robo_trace::store::Persistor::Ptr& persistor, const robo_trace::store::Container::Ptr& metadata) 
: m_persistor(persistor) {


/*
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
*/
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
        Push the message to Persistor.
    */

    bsoncxx::document::value entry = builder.extract();
    m_persistor->store(entry);

/*
    auto client = robo_trace::store::Connector::instance().getClient();
    bsoncxx::stdx::optional<mongocxx::result::insert_one> result = (*client)[m_database_name][m_collection_name_data].insert_one(builder.view());

#ifdef PROCESSING_MODULE_BASIC_STORE_VALIDATE_MESSAGE_WRITEBACK
    if (!result || result.value().result().inserted_count() != 1) {
        throw std::runtime_error("Failed storing message for collection " + m_collection_name_data + "!");
    }
#endif
*/

}



}