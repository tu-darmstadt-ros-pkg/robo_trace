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
#include <memory>
// Mongo
#include "robo_trace/config.h"
#include <mongo/client/dbclient.h>
// Project
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/stage/stage.hpp"


namespace robo_trace {

/**
 * TODO: Still not adjusted for the new mongocxx driver! 
 * 
 * 
 * 
 */
class BlobDecouplingForwardStage final : public ProcessingStage { 

public:

    /**
     * 
     */
    BlobDecouplingForwardStage(const std::shared_ptr<mongo::DBClientConnection>& connection, const std::string& database);
  
    /**
     * 
     */
    virtual ~BlobDecouplingForwardStage();
    
    /**
     *
     */
    virtual ProcessingMode getMode() const final override;
    
    /**
     * 
     */
    virtual void process(const ProcessingContext::Ptr& context) final override;

private:

    /**
     * 
     */
    void decouple(mongo::BSONObjBuilder& builder_decoupled, const mongo::BSONObj& serialized);

private:

    /** */
    const std::string m_database_name;
    /** */
    const std::shared_ptr<mongo::DBClientConnection> m_connection;
    /** */
    const std::shared_ptr<mongo::GridFS> m_grid_filesystem;

};

}