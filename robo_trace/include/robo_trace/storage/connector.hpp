#pragma once
// Std
#include <mutex>
// MongoDB
#include "robo_trace/config.h"
#include <mongo/client/init.h>
#include <mongo/client/dbclient.h>
// Project
#include "robo_trace/storage/options.hpp"


namespace robo_trace {

class ConnectionProvider {

public:

    /**
     *
     */
    static void initialize();

    /**
     * 
     */
    static std::shared_ptr<mongo::DBClientConnection> getConnection(const ConnectionOptions::ConstPtr& options);

};


}