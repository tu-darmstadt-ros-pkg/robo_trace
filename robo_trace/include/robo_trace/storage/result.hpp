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
#include <string>
#include <optional>
// MongoDB
#include "robo_trace/config.h"
#include <mongo/bson/bsonobj.h>
#include <mongo/bson/bsonobjbuilder.h>
#include <mongo/client/dbclientcursor.h>


namespace robo_trace {

class MessageQueryResultIterator {

public:

    typedef std::shared_ptr<MessageQueryResultIterator> Ptr;
    typedef std::shared_ptr<const MessageQueryResultIterator> ConstPtr;

public:

    /**
     *
     */
    MessageQueryResultIterator(const std::shared_ptr<mongo::DBClientConnection>& connection, const mongo::Query& query, const std::string& path);

    /**
     *
     */
    ~MessageQueryResultIterator();
    
    /**
     *
     */
    bool isValid();

    /**
     *
     */
    bool isCompleted();

    /**
     *
     */
    void next();

    /**
     *
     */
    const mongo::BSONObj getMessage() const;

    /**
     *
     */
    const mongo::BSONObj getMetadata() const;

private:

    /** The cursor over the result data. */
    std::unique_ptr<mongo::DBClientCursor> m_result_cursor;    
    /** The optional next element. */
    std::optional<mongo::BSONObj> m_next_element;

};

}