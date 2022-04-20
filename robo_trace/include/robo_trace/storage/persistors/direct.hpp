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
#include <string>
// MongoCXX
#include <mongocxx/options/insert.hpp>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/storage/persistor.hpp"


namespace robo_trace::store {

class DirectPersistor final : public Persistor {

public:

    /**
     *
     */
    DirectPersistor(const std::string& database, const std::string& collection);

    /**
     *
     */
    ~DirectPersistor();
    
    /**
     *
     */
    virtual void store(bsoncxx::document::value& element) final override;

private:

    /** */
    mongocxx::options::insert m_insert_option;
    
};

}