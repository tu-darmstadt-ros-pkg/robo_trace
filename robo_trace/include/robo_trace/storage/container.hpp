#pragma once

// Std
#include <string>
#include <memory>
// MongoDB
#include "robo_trace/config.h"
#include <mongo/bson/bsonobj.h>
#include <mongo/bson/bsonobjbuilder.h>


namespace robo_trace {

class DataContainer {

public:

    typedef std::shared_ptr<DataContainer> Ptr;

    typedef std::shared_ptr<const DataContainer> ConstPtr;

public:

    /**
     *
     */    
    DataContainer();

    /**
     *
     */
    DataContainer(const mongo::BSONObj& other);

    /**
     *
     */
    ~DataContainer();


    void append(const std::string& name, const bool val);

    bool getBool(const std::string& name);


    void append(const std::string& name, const int val);
    
    int getInt(const std::string& name);


    void append(const std::string& name, const double val);

    double getDouble(const std::string& name);


    void append(const std::string& name, const std::string val);

    const std::string getString(const std::string& name);


    void append(const std::string& name, const void* data, size_t size);

    const void* getBinData(const std::string& name, size_t& size); 


    /**
     * Returns thet metadata container associated to the given key. A 
     * corresponding container will be created if not present.
     * 
     * @param name the key under which the nested container is stored.
     * @return the nested container store.
     */
    const DataContainer::Ptr getContainer(const std::string& name);

     /**
     * 
     */
    void serialize(mongo::BSONObjBuilder& builder);

private:

    /**
     * 
     */
    void synchronize();

private:

    /** */
    mongo::BSONObjBuilder m_bson_builder;
    
    /** */
    bool m_read_dirty_flag;
    /** */
    mongo::BSONObj m_read_object;

    /**  */
    std::unordered_map<std::string, DataContainer::Ptr> m_child_wrappers;

};

}