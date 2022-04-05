// Base
#include "robo_trace/storage/persistors/direct.hpp"
// Std
#include <stdexcept>
#ifdef EVALUATION_CAPTURE_WRITEBACK_TIME
#include <chrono>
#endif
// MongoCXX
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
#include <bsoncxx/builder/basic/array.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
#include <bsoncxx/types.hpp>
// Project
#include "robo_trace/storage/connector.hpp"


namespace robo_trace::store {

DirectPersistor::DirectPersistor(const std::string& database, const std::string& collection)
: Persistor(database, collection) {
    
#ifdef PERSISTOR_BYPASS_DOCUMENT_VALIDATION
    m_insert_option.bypass_document_validation(true);
#endif

    Persistor::setup();

}

DirectPersistor::~DirectPersistor() = default;

void DirectPersistor::store(bsoncxx::document::value& element) {

    auto client = robo_trace::store::Connector::instance().getClient();
    auto database = (*client)[m_database];

    bsoncxx::stdx::optional<mongocxx::result::insert_one> result = database[m_collection].insert_one(std::move(element), m_insert_option);

#ifdef PERSISTOR_VALIDATE_MESSAGE_WRITEBACK
    if (!result || result.value().result().inserted_count() != 1) {
        throw std::runtime_error("Failed storing message for collection " + m_collection + "!");
    }
#endif

}

}