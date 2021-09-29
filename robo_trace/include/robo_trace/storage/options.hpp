#pragma once

// Std
#include <string>
#include <memory>
// Project
#include "robo_trace/util/options.hpp"


namespace robo_trace {

class ConnectionOptions final : public OptionsContainer {

public:

    typedef std::shared_ptr<ConnectionOptions> Ptr;
    typedef std::shared_ptr<const ConnectionOptions> ConstPtr;

public:

    /**
     *
     */
    ConnectionOptions();

    /**
     *
     */
    virtual ~ConnectionOptions(); 

    /**
     *
     */
    virtual void load(const ros::NodeHandle& node_handle) final override;

    /**
     *
     */
    virtual void setup(po::options_description& description) final override;

    /**
     *
     */
    virtual void load(po::variables_map& options) final override;
    
    /**
     *
     */
    virtual void validate() final override;

public:

    /** */
    std::string m_database_name;
    /** */
    std::string m_summary_collection_name;

    /** */
    int m_database_server_port;
    /** */
    std::string m_database_server_host;
    /** */
    float m_database_server_connection_timeout;

};

}