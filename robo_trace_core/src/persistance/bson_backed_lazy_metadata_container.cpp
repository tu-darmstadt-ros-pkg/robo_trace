// Base
#include "robo_trace_core/persistance/bson_backed_lazy_metadata_container.hpp"
#include <ros/console.h>

namespace robo_trace {

LazyMongoBsonMetadataContainer::LazyMongoBsonMetadataContainer()
: m_read_dirty_flag(false) {
    //
};

LazyMongoBsonMetadataContainer::~LazyMongoBsonMetadataContainer() = default;


bool LazyMongoBsonMetadataContainer::isDirty() const {
    return m_read_dirty_flag;
}


void LazyMongoBsonMetadataContainer::append(const std::string& name, const bool val) {
    m_bson_builder.append(name, val);
    m_read_dirty_flag = true;
}

bool LazyMongoBsonMetadataContainer::getBool(const std::string& name) {

    if (m_read_dirty_flag) {
        synchronize();
    }
    
    return m_read_object.getField(name).boolean();
}


void LazyMongoBsonMetadataContainer::append(const std::string& name, const int val) {
    m_bson_builder.append(name, val);
    m_read_dirty_flag = true;
}

int LazyMongoBsonMetadataContainer::getInt(const std::string& name) {

    if (m_read_dirty_flag) {
        synchronize();
    }

    return m_read_object.getField(name).numberInt();
}


void LazyMongoBsonMetadataContainer::append(const std::string& name, const double val) {
    m_bson_builder.append(name, val);
    m_read_dirty_flag = true;
}

double LazyMongoBsonMetadataContainer::getDouble(const std::string& name) {

    if (m_read_dirty_flag) {
        synchronize();
    }

    return m_read_object.getField(name).numberDouble();
}


void LazyMongoBsonMetadataContainer::append(const std::string& name, const std::string val) {
    m_bson_builder.append(name, val);
    m_read_dirty_flag = true;
}

const std::string LazyMongoBsonMetadataContainer::getString(const std::string& name) {

    if (m_read_dirty_flag) {
        synchronize();
    }

    return m_read_object.getField(name).str();
}

void LazyMongoBsonMetadataContainer::append(const std::string& name, const void* data, size_t size) {
    m_bson_builder.appendBinData(name, size, mongo::BinDataType::BinDataGeneral, data);
    m_read_dirty_flag = true;
}

const void* LazyMongoBsonMetadataContainer::getBinData(const std::string& name, size_t& size) {

    if (m_read_dirty_flag) {
        synchronize();
    }

    int size_i;
    const char* data = m_read_object.getField(name).binData(size_i);

    size = static_cast<size_t>(size_i);
    return data;
}


const MetadataContainer::Ptr LazyMongoBsonMetadataContainer::getContainer(const std::string& name) {
    
    /*
        Note that we can ONLY append to a bson builder! Hence, when nesting containers
        we need to must create the components individually and then merge them on serialization.
    */

    std::unordered_map<std::string, LazyMongoBsonMetadataContainer::Ptr>::const_iterator result = m_child_wrappers.find(name);
    
    if (result != m_child_wrappers.end()) {
        return result->second;
    }

    LazyMongoBsonMetadataContainer::Ptr created = std::make_shared<LazyMongoBsonMetadataContainer>();
    // Need to make a copy of name here as it's only a reference.
    // child_wrappers[name] = created;
    //std::string x = name;
    //std::pair<std::string, MongoBsonMetadataContainer::Ptr> pair = std::make_pair<std::string, MongoBsonMetadataContainer::Ptr>(x, created);
    m_child_wrappers[name] = created;

    return std::static_pointer_cast<MetadataContainer>(created);

}

void LazyMongoBsonMetadataContainer::synchronize() {
    // Simply load in the new tmp object.
    m_read_object = m_bson_builder.asTempObj();
    m_read_dirty_flag = false;
}

void LazyMongoBsonMetadataContainer::serialize(mongo::BSONObjBuilder& builder) {

    builder.appendElements(m_bson_builder.obj());

    for (auto& entry : m_child_wrappers) {

        // Builder appending to current byte array  
        mongo::BufBuilder& sub_object_buffer_start = builder.subobjStart(entry.first);
        mongo::BSONObjBuilder sub_object_builder(sub_object_buffer_start);

        entry.second->serialize(sub_object_builder);
        // Sub builder finished.
        sub_object_builder.done();

    }

}

}