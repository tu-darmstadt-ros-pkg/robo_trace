/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/storage/connector.hpp"
// Std
#include <string>
#include <sstream>
#include <stdexcept>
#include <iostream>
// MongoDB
#include <bsoncxx/stdx/make_unique.hpp>
// Ros
#include <ros/console.h>


namespace robo_trace::store {

Connector& Connector::instance() {
    
    static Connector instance;

    return instance;
}

Connector::Connector() = default;

Connector::~Connector() = default;

void Connector::configure(const Options::ConstPtr& options) {
    
    m_driver_instance = bsoncxx::stdx::make_unique<mongocxx::instance>();

    std::stringstream cs;
    // Database location
    cs << "mongodb://" << options->m_host_name << ":" << std::to_string(options->m_host_port) << "/?";
    // Pool size
    cs << "minPoolSize=" << std::to_string(options->m_connection_pool_size_min) << "&";
    cs << "maxPoolSize=" << std::to_string(options->m_connection_pool_size_max) << "&";
    // Timeout
    cs << "serverSelectionTimeoutMS=" << std::to_string(options->m_connection_timeout_ms);
    
    std::cout << cs.str() << std::endl;
    const mongocxx::uri uri = mongocxx::uri{cs.str()};
    m_pool = bsoncxx::stdx::make_unique<mongocxx::pool>(std::move(uri));

}

mongocxx::pool::entry Connector::getClient() {
    return m_pool->acquire();
}

}
