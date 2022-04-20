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
#include <unordered_map>
// MongoDB
#include "robo_trace/config.h"
#include <mongo/bson/bsonobj.h>
#include <mongo/bson/bsonobjbuilder.h>


namespace robo_trace {

class MessageQuery {

public:

    typedef std::shared_ptr<MessageQuery> Ptr;
    typedef std::shared_ptr<const MessageQuery> ConstPtr;

public:

    /**
     *
     */
    MessageQuery();

    /**
     *
     */
    ~MessageQuery();

    /**
     *
     */ 
    void setFieldEquals(const std::string& field, const std::string& value);

    /**
     *
     */  
    void setFieldEquals(const std::string& field, const int value);

    /**
     *
     */  
    void setFieldEquals(const std::string& field, const bool value);

    /**
     *
     */  
    void setFieldLessThan(const std::string& field, const double value);

    /**
     *
     */  
    void setFieldLessThan(const std::string& field, const int value);

    /**
     *
     */  
    void setFieldEqualLessThan(const std::string& field, const double value);

    /**
     *
     */  
    void setFieldEqualLessThan(const std::string& field, const int value);

    /**
     *
     */  
    void setFieldGreaterThan(const std::string& field, const double value);

    /**
     *
     */  
    void setFieldGreaterThan(const std::string& field, const int value);

    /**
     *
     */  
    void setFieldEqualGreaterThan(const std::string& field, const double value);

    /**
     *
     */  
    void setFieldEqualGreaterThan(const std::string& field, const int value);

    /**
     *
     */  
    void setFieldInRangeExclusive(const std::string& field, const double lower_bound, const double upper_bound);

    /**
     *
     */  
    void setFieldInRangeExclusive(const std::string& field, const int lower_bound, const int upper_bound);

    /**
     *
     */  
    void setFieldInRangeInclusive(const std::string& field, const double lower_bound, const double upper_bound);

    /**
     *
     */  
    void setFieldInRangeInclusive(const std::string& field, const int lower_bound, const int upper_bound);

    /**
     * 
     */
    const MessageQuery::Ptr getQuery(const std::string& name);

    /**
     * 
     */
    void serialize(mongo::BSONObjBuilder& builder);

private:

    /** */
    mongo::BSONObjBuilder m_bson_builder;
    /**  */
    std::unordered_map<std::string, MessageQuery::Ptr> m_sub_queries;


};

}