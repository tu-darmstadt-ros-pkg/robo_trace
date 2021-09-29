// Base
#include "robo_trace/storage/connector.hpp"
// Std
#include <string>
#include <stdexcept>
// Ros
#include <ros/console.h>


namespace robo_trace {


void ConnectionProvider::initialize() {
    
    /*
        Simply initializes the mongo driver.    
    */
    
    mongo::client::initialize();

}

std::shared_ptr<mongo::DBClientConnection> ConnectionProvider::getConnection(const ConnectionOptions::ConstPtr& options) {

    const std::string database_address = options->m_database_server_host + ":" + std::to_string(options->m_database_server_port); 
    const ros::WallTime connect_timeout = ros::WallTime::now() + ros::WallDuration(options->m_database_server_connection_timeout);
    
    std::shared_ptr<mongo::DBClientConnection> connection;

    while (ros::ok() && ros::WallTime::now() < connect_timeout) {

        connection.reset(new mongo::DBClientConnection());
        
        try {
            
            connection->connect(database_address);
            
            if (!connection->isFailed()) {
                break;
            }

        } catch (mongo::ConnectException& e) {
            ros::Duration(1.0).sleep();
        }

    }

    if (!connection || connection->isFailed()) {
        throw std::runtime_error("Failed connecting to database.");
    }

    return connection;

}

}
