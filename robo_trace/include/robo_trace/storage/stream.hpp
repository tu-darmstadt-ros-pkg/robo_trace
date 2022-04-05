#pragma once
// Std
#include <cstdint>
#include <string>
#include <memory>
// MongoCXX
#include <bsoncxx/types/bson_value/value.hpp>
#include <mongocxx/options/gridfs/bucket.hpp>


namespace robo_trace::store {
    
class StreamHandler {

public:

    /** */
    typedef std::shared_ptr<StreamHandler> Ptr;
    /** */
    typedef std::shared_ptr<const StreamHandler> ConstPtr;

public:
  
    /**
     * 
     */
    StreamHandler(const std::string& database, const std::string& bucket);

    /**
     * 
     */
    ~StreamHandler();

    /**
     * 
     */
    std::string getDatabase() const;

    /**
     * 
     */
    void setDatabase(const std::string& database);

    /**
     * 
     */
    const bsoncxx::types::bson_value::value store(const std::string& name, const uint8_t* data, const size_t size);


private:

    /** */
    std::string m_database;

    /** */
    mongocxx::options::gridfs::bucket m_bucket_options;

};

}