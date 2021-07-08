// Base
#include "robo_trace_core/persistance/connection.hpp"
// Ros
#include <ros/console.h>
// Project
#include "robo_trace_core/persistance/store.hpp"


namespace robo_trace {

MongoDBConnection::MongoDBConnection(const std::string& name) 
: m_name(name) {
    //
}

MongoDBConnection::~MongoDBConnection() = default;


const std::string& MongoDBConnection::getName() const {
    return m_name;
}

const int MongoDBConnection::getPort() const {
    return m_port;
}

const std::string MongoDBConnection::getHost() const {
    return m_host;
}


ros::NodeHandle& MongoDBConnection::getSystemNodeHandle() {
    return m_system_node_handle;
}

ros::NodeHandle& MongoDBConnection::getConnectionNodeHandle() {
    return m_connection_node_handle;
}


const std::shared_ptr<mongo::DBClientConnection>& MongoDBConnection::getConnection() const {
    return m_database_connection;
}

MessageStore::Ptr MongoDBConnection::getStore(const std::string& topic) {
    return std::make_shared<MessageStore>(*this, "robo_trace", topic);
}

void MongoDBConnection::initialize(ros::NodeHandle& system_node_handle) {

    /*
        Initialize node handles for this plugin.
    */

    m_system_node_handle = system_node_handle;
    m_connection_node_handle = ros::NodeHandle(m_system_node_handle, "connection");
    ROS_INFO_STREAM("Connection node handle NS: " << m_connection_node_handle.getNamespace());

    /*
        Load parameters from Ros.
    */

    //const ros::NodeHandle connection_ns = ros::NodeHandle(m_private_handle, "connection");
    m_connection_node_handle.getParam("port", m_port);
    m_connection_node_handle.getParam("host", m_host);
    m_connection_node_handle.getParam("attempts", m_attempts);

    /*
        Initialize database connection. This section is heavily inspired by warehouse_ros_mong:
          -> https://github.com/ros-planning/warehouse_ros_mongo/blob/melodic-devel/src/database_connection.cpp#L72

    */

    mongo::client::initialize();

    const std::string db_address = m_host + ":" + std::to_string(m_port);
    ROS_INFO_STREAM("Connecting to MongoDB at: " << db_address);

    for (uint32_t attempt = 0; attempt < m_attempts; ++attempt) {

        m_database_connection.reset(new mongo::DBClientConnection());
        
        try {

            // ROS_DEBUG_STREAM_NAMED("db_connect", "Attempting to connect to MongoDB at " << db_address);
            m_database_connection->connect(db_address);
            
            if (!m_database_connection->isFailed()) {
                break;
            }

        } catch (mongo::ConnectException& e) {
            ros::Duration(1.0).sleep();
        }

    }
    ROS_INFO_STREAM("Connected: " << !m_database_connection->isFailed());

}

}
