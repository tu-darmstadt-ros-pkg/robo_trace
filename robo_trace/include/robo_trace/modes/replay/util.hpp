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