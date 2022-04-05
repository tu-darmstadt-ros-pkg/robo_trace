// Base
#include "robo_trace/storage/stream.hpp"
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
    
StreamHandler::StreamHandler(const std::string& database, const std::string& bucket) 
: m_database(database) {

    // Assuming the collection name is unique. 
    m_bucket_options.bucket_name(bucket);

}

StreamHandler::~StreamHandler() = default;

std::string StreamHandler::getDatabase() const {
    return m_database;
}

void StreamHandler::setDatabase(const std::string& database) {
    m_database = database;
}

const bsoncxx::types::bson_value::value StreamHandler::store(const std::string& name, const uint8_t* data, const size_t size) {

    auto client = robo_trace::store::Connector::instance().getClient();    
    auto database = (*client)[m_database];

    auto bucket = database.gridfs_bucket(m_bucket_options);
    auto uploader = bucket.open_upload_stream(name);
    
    // TODO: Might throw a bunch of fancy exceptions.
    uploader.write(data, size);
    
    auto result = uploader.close();

    bsoncxx::types::bson_value::value owning(result.id());
    return owning;

}
} 
