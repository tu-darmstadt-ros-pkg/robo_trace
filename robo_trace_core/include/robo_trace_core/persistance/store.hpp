#pragma once
// Std
#include <string>
#include <memory>
// Project
#include "robo_trace_plugin_interface/processing/metadata.hpp"
#include "robo_trace_plugin_interface/processing/context.hpp"


namespace robo_trace {

// Forward declaration
class MongoDBConnection;

class MessageStore {

public:

    typedef std::shared_ptr<MessageStore> Ptr;
    typedef std::shared_ptr<const MessageStore> ConstPtr;

public:

    /**
     * 
     */
    MessageStore(const MongoDBConnection& connection, const std::string& database, const std::string& collection);

    /**
     * 
     */
    ~MessageStore();

    /**
     * 
     */
    const std::string& getDatabaseName() const;

    /**
     * 
     */
    const std::string& getCollectionName() const;

    /**
     * 
     */
    const std::string& getStorePath() const;

    /**
     * 
     */
    MetadataContainer::Ptr getContainer();

    /**
     * 
     */
    void store(const MessageProcessingContext::Ptr& context);

    /**
     * 
     */
    void store(const MetadataContainer::Ptr& metadata);

private:

    /** */
    const std::string m_database;
    /** */
    const std::string m_collection;
    /** */
    const std::string m_store_path;

    /** */
    const MongoDBConnection& m_connection;
    
};

}