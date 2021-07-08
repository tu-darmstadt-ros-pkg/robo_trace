#pragma once

// Std
#include <malloc.h>
#include <memory>
#include "robo_trace_core/config.h"
// MongoDB
#include <mongo/db/json.h>


namespace robo_trace {

class WrappedBSON : public  mongo::BSONObj {

public:

    /**
     * 
     */
    WrappedBSON();

    /**
     * 
     */
    WrappedBSON(const WrappedBSON& other);

    /**
     * 
     */
    WrappedBSON(const BSONObj& other);

    /**
     * 
     */
    WrappedBSON(const std::string& json);

protected:

    /**
     * 
     */
    void update();

protected:

    /** */
    std::shared_ptr<mongo::BSONObjBuilder> m_builder;

};

}