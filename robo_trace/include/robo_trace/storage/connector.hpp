/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
#pragma once
// Std
#include <mutex>
#include <memory>
// MongoDB
#include <mongocxx/pool.hpp>
#include <mongocxx/instance.hpp>
// Project
#include "robo_trace/storage/options.hpp"


namespace robo_trace::store {

class Connector {

public:

    /**
     * 
     */
    static Connector& instance();

private:

    /**
     * 
     */
    Connector();

    /**
     * 
     */
    ~Connector();

public:

    /**
     *
     */
    void configure(const Options::ConstPtr& options);

    /**
     * 
     */
    mongocxx::pool::entry getClient();

private:

    /** */
    std::unique_ptr<mongocxx::instance> m_driver_instance = nullptr;
    /** */
    std::unique_ptr<mongocxx::pool> m_pool = nullptr;

};


}