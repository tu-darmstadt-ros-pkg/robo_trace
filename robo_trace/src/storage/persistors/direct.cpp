// Base
#include "robo_trace/storage/persistors/direct.hpp"
// MongoCXX
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
// Project
#include "robo_trace/parameters.hpp"


namespace robo_trace::store {

BasicPersistor::BasicPersistor(const std::string& database, const std::string& collection)
: m_database(database), m_collection(collection) {
    
    mongocxx::client client = robo_trace::store::Connector::instance().getClient();
    mongocxx::database database = (*client)[database];
    
    if (database.has_collecion(collection)) {
        return;
    }

    mongo_database.create_collection(collection);

}

BasicPersistor::~BasicPersistor() = default;

void BasicPersistor::store(const bsoncxx::document::view& element) final {

    mongocxx::client mongo_client = robo_trace::store::Connector::instance().getClient();
    bsoncxx::stdx::optional<mongocxx::result::insert_one> result = (*client)[m_database][m_collection].insert_one(element);


#ifdef PROCESSING_MODULE_BASIC_STORE_VALIDATE_MESSAGE_WRITEBACK
    if (!result || result.value().result().inserted_count() != 1) {
        throw std::runtime_exception("Failed storing message for collection " + m_collection_name_data + "!");
    }
#endif


}

}