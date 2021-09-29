// Base
#include "robo_trace/storage/options.hpp"
// Std
#include <sstream>


namespace robo_trace {


ConnectionOptions::ConnectionOptions() 
: m_database_name("robo_trace"), 
  m_database_server_port(27017), 
  m_database_server_host("localhost"), 
  m_database_server_connection_timeout(10.0) {
      // 
};

ConnectionOptions::~ConnectionOptions() = default;


void ConnectionOptions::load(const ros::NodeHandle& node_handle) {

    ros::NodeHandle connection_namespace(node_handle, "connection");
    
    connection_namespace.getParam("database", m_database_name); 
    connection_namespace.getParam("summary_collection", m_summary_collection_name);
    connection_namespace.getParam("port", m_database_server_port);
    connection_namespace.getParam("host", m_database_server_host);
    connection_namespace.getParam("timeout", m_database_server_connection_timeout);

}
 
void ConnectionOptions::setup(po::options_description& description) {

    description.add_options()
        ("database_name", po::value<std::string>(), "The name of the database to store the recording in.")
        ("summary_collection_name", po::value<std::string>(), "The name of the summary collection for the recording.")
        ("database_server_host", po::value<std::string>(), "The host name of the database server")
        ("database_server_port", po::value<int>(), "The port of the database server")
        ("database_server_timeout", po::value<float>(), "The timeout when communicating with the database server");

}

void ConnectionOptions::load(po::variables_map& options) {

    if (options.count("database_name")) {
        m_database_name = options["database_name"].as<std::string>();
    }

    if (options.count("database_server_host")) {
        m_database_server_host = options["database_server_host"].as<std::string>();
    }

    if (options.count("database_server_port")) {
        m_database_server_port = options["database_server_port"].as<int>();
    }

    if (options.count("database_server_timeout")) {
        m_database_server_connection_timeout = options["database_server_timeout"].as<float>();
    }

    if (options.count("summary_collection_name")) {
        m_summary_collection_name = options["summary_collection_name"].as<std::string>();
    }

}

void ConnectionOptions::validate() {

    if (m_database_name == "") {
        std::cout << "A empty database name is not valid." << std::endl; exit(0);
    }

    if (m_database_server_host == "") {
        std::cout << "A empty database server host name is not valid." << std::endl; exit(0);
    }

    if (m_summary_collection_name == "") {
        std::cout << "A empty summary collection name is not valid." << std::endl; exit(0);
    }

    if (m_database_server_port == 0) {
        std::cout << "A empty database server port name is not valid." << std::endl; exit(0);
    }

}


}