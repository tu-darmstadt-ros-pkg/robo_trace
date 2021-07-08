#pragma once
// Std
#include <memory>
#include <string>
// Ros
#include <ros/ros.h>
// MongoDB
#include "robo_trace_core/config.h"
#include <mongo/client/init.h>
#include <mongo/client/dbclient.h>
// Project
#include "robo_trace_core/persistance/store.hpp"


namespace robo_trace {

class MongoDBConnection {

public:

    typedef std::shared_ptr<MongoDBConnection> Ptr;
    typedef std::shared_ptr<const MongoDBConnection> ConstPtr;

public:

    /**
     * 
     */
    MongoDBConnection(const std::string& name);

    /**
     * 
     */
    ~MongoDBConnection();

    /**
     * 
     */
    const std::string& getName() const;

    /**
     * 
     */
    const int getPort() const;

    /**
     * 
     */
    const std::string getHost() const;

    /**
     * 
     */
    ros::NodeHandle& getSystemNodeHandle();

    /**
     * 
     */
    ros::NodeHandle& getConnectionNodeHandle();

    /**
     * 
     */
    const std::shared_ptr<mongo::DBClientConnection>& getConnection() const;

    /**
     * 
     */
    MessageStore::Ptr getStore(const std::string& topic);  

    /**
     * 
     */
    void initialize(ros::NodeHandle& system_node_handle);

private:
    
    /** */
    const std::string m_name;

    /** The port of the database. */
    int m_port;
    /** The address of the database. */
    std::string m_host;
    /** */
    int m_attempts;

    /** */
    ros::NodeHandle m_system_node_handle;
    /** */
    ros::NodeHandle m_connection_node_handle;

    /** The connection to the mongo db database. */
    std::shared_ptr<mongo::DBClientConnection> m_database_connection;

};


}