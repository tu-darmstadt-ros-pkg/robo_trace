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
// MongoCXX
#include <bsoncxx/document/value.hpp>


namespace robo_trace::store {

class Persistor {

public:

    typedef std::shared_ptr<Persistor> Ptr;

    typedef std::shared_ptr<const Persistor> ConstPtr;

public:

    /**
     *
     */
    Persistor(const std::string& database, const std::string& collection);

    /**
     *
     */
    virtual ~Persistor();

    /**
     * Returns the name of the database this persistor is
     * peristing to.
     * 
     * @return the name of the database elements are store to.
     */
    const std::string& getDatabase();

    /**
     * Returns the name of the collection this persisor is
     * persisting to.
     * 
     * @return the name of the collection elements are stored to.
     */
    const std::string& getCollection();

    /**
     *
     */
    void setIndex(const std::string& field, const bool unique);

    /**
     *
     */
    virtual void flush();

    /**
     *
     */
    virtual void store(bsoncxx::document::value& element) = 0;

protected:

    /**
     * 
     */
    void setup();

protected:

    /** */
    const std::string m_database;
    /** */
    const std::string m_collection;

};

}