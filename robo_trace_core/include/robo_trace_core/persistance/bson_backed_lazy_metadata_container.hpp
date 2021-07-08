#pragma once

// Std
#include <memory>
#include <string>
#include <unordered_map>
// MongoDB
#include "robo_trace_core/config.h"
#include <mongo/bson/bsonobj.h>
#include <mongo/bson/bsonobjbuilder.h>
// Project
#include "robo_trace_plugin_interface/processing/metadata.hpp"


namespace robo_trace {

class LazyMongoBsonMetadataContainer final : public MetadataContainer {

public:

    typedef std::shared_ptr<LazyMongoBsonMetadataContainer> Ptr;

    typedef std::shared_ptr<const LazyMongoBsonMetadataContainer> ConstPtr;

public:


    LazyMongoBsonMetadataContainer();

    virtual ~LazyMongoBsonMetadataContainer();


    virtual void append(const std::string& name, const bool val) final override;

    virtual bool getBool(const std::string& name) final override;


    virtual void append(const std::string& name, const int val) final override;
    
    virtual int getInt(const std::string& name) final override;


    virtual void append(const std::string& name, const double val) final override;

    virtual double getDouble(const std::string& name) final override;


    virtual void append(const std::string& name, const std::string val) final override;

    virtual const std::string getString(const std::string& name) final override;


    virtual void append(const std::string& name, const void* data, size_t size) final override;

    virtual const void* getBinData(const std::string& name, size_t& size) final override; 


    /**
     * 
     */
    virtual const MetadataContainer::Ptr getContainer(const std::string& name) final override;

    /**
     * 
     */
    bool isDirty() const;

    /**
     * 
     */
    void synchronize();

    /**
     * 
     */
    void serialize(mongo::BSONObjBuilder& builder);

private:

    /** */
    mongo::BSONObjBuilder m_bson_builder;
    
    /** */
    bool m_read_dirty_flag;
    /** */
    mongo::BSONObj m_read_object;

    /**  */
    std::unordered_map<std::string, LazyMongoBsonMetadataContainer::Ptr> m_child_wrappers;

    

};

}