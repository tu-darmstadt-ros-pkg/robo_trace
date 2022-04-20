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
#include <memory>
#include <optional>
// MongoDB
#include "robo_trace/config.h"
#include <mongo/client/dbclient.h>
#include <mongo/client/dbclientinterface.h>
// Project
#include "robo_trace/storage/options.hpp"


namespace robo_trace:: {

class ReplayUtil {

public:

    /**
     *
     */
    static std::optional<double> getRecordStartTime(const ConnectionOptions::Ptr connection_options, const std::shared_ptr<mongo::DBClientConnection> connection);

};

}