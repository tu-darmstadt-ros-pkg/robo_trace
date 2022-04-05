#include "robo_trace/storage/persistor.hpp"
// Std
#include <stdexcept>
// MongoCXX
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/gridfs/bucket.hpp>
#include <mongocxx/gridfs/uploader.hpp>
#include <mongocxx/result/gridfs/upload.hpp>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/storage/connector.hpp"


namespace robo_trace::store {

Persistor::Persistor(const std::string& database, const std::string& collection)
: m_database(database), m_collection(collection) {

};

Persistor::~Persistor() = default;

const std::string& Persistor::getDatabase() {
    return m_database;
}

const std::string& Persistor::getCollection() {
    return m_collection;
}

void Persistor::setup() {

    auto client = robo_trace::store::Connector::instance().getClient();
    mongocxx::database mongo_database = (*client)[m_database];
    
    if (!mongo_database.has_collection(m_collection)) {
        mongo_database.create_collection(m_collection);
    } else {
#ifdef PERSISTOR_FAIL_IF_DATA_COLLECTION_PRESENT
        throw std::runtime_error("Collection already present for '" + m_collection + "'!");
#endif
    }
    
}

void Persistor::setIndex(const std::string& field, const bool unique) {

    auto client = robo_trace::store::Connector::instance().getClient();
    mongocxx::collection collection = (*client)[m_database][m_collection];
    
    mongocxx::options::index collection_index_options{};
    collection_index_options.unique(unique);

    collection.create_index(
        // keys
        bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp(field, 1)),
        // index_options
        collection_index_options
    );

}

void Persistor::flush() {
    //
}

}