// Base
#include "robo_trace/storage/options.hpp"
// Std
#include <sstream>


namespace robo_trace::store {

Options::Options() 
: m_host_port(27017), 
  m_host_name("localhost"),
  m_database_name("robo_trace"),
  m_collection_name_summary("__meta__"),
  m_connection_pool_size_min(2),
  m_connection_pool_size_max(10),
  m_connection_timeout_ms(100) {
      // 
};

Options::~Options() = default;


void Options::load(const ros::NodeHandle& node_handle) {

    ros::NodeHandle connection_namespace(node_handle, "connection");
    
    connection_namespace.getParam(ros::names::append("host", "port"), m_host_port);
    connection_namespace.getParam(ros::names::append("host", "name"), m_host_name);

    connection_namespace.getParam(ros::names::append("database", "name"), m_database_name); 

    connection_namespace.getParam(ros::names::append("collections", "meta"), m_collection_name_summary);

    connection_namespace.getParam(ros::names::append("options", "pool_size_min"), m_connection_pool_size_min);
    connection_namespace.getParam(ros::names::append("options", "pool_size_max"), m_connection_pool_size_max);
    connection_namespace.getParam(ros::names::append("options", "timeout"), m_connection_timeout_ms);


}
 
void Options::setup(po::options_description& description) {

    description.add_options()
        ("db_name", po::value<std::string>(), "The name of the database to store the recording in.")
        ("db_meta_collection", po::value<std::string>(), "The name of the meta collection for the recording.")
        ("db_host_name", po::value<std::string>(), "The host name of the database server")
        ("db_host_port", po::value<int>(), "The port of the database server")
        ("db_timeout", po::value<float>(), "The timeout when communicating with the database server");

}

void Options::load(po::variables_map& options) {

    if (options.count("db_name")) {
        m_database_name = options["db_name"].as<std::string>();
    }

    if (options.count("db_host_name")) {
        m_host_name = options["db_host_name"].as<std::string>();
    }

    if (options.count("db_host_port")) {
        m_host_port = options["db_host_port"].as<int>();
    }

    if (options.count("db_timeout")) {
        m_connection_timeout_ms = options["db_timeout"].as<float>();
    }

    if (options.count("db_summary_collection")) {
        m_collection_name_summary = options["db_summary_collection"].as<std::string>();
    }

}

void Options::validate() {

    if (m_database_name == "") {
        std::cout << "Please provide a valid database name." << std::endl; exit(0);
    }

    if (m_host_name == "") {
        std::cout << "Please provide the hostname for the database server." << std::endl; exit(0);
    }
    
    if (m_host_port == 0) {
        std::cout << "Please provide a valid prot for the database server." << std::endl; exit(0);
    }

    if (m_collection_name_summary == "") {
        std::cout << "Please provide a valid name for the summary collection." << std::endl; exit(0);
    }

}


}