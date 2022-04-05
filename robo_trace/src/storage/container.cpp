// Base
#include "robo_trace/storage/container.hpp"
// MongoDB
#include <bsoncxx/types.hpp>
#include <bsoncxx/types/bson_value/view.hpp>
#include <bsoncxx/stdx/string_view.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>


namespace robo_trace::store {

Container::Container()
: m_read_view_dirty(false) {
    //
}

/*
DataContainer::DataContainer(const mongo::BSONObj& other)
: m_read_dirty_flag(true) {

    for(mongo::BSONObj::iterator iterator = other.begin(); iterator.more(); ) { 

        const mongo::BSONElement& element = iterator.next();
    
        if (element.type() == mongo::Object) {
            const std::string name(element.fieldName(), element.fieldNameSize());
            m_child_wrappers[name] = std::make_shared<DataContainer>(element.Obj());
        } else {
            m_bson_builder.append(element);
        }

    }

}*/

Container::Container(const bsoncxx::document::view& other)
: m_read_view_dirty(true) {

    //for(bsoncxx::document::view::const_iterator iterator = other.begin(); iterator.more(); ) { 
    for (const bsoncxx::document::element& element : other) {
        //const bsoncxx::document::element& element = iterator.next();
    
        if (element.type() == bsoncxx::type::k_document) {
            m_children[element.key().to_string()] = std::make_shared<Container>(element.get_document());
        } else {
            m_builder.append(bsoncxx::builder::basic::kvp(element.key(), element.get_value()));
        }

    }

}

Container::~Container() = default;


void Container::append(const std::string& name, bool val) {
    m_builder.append(bsoncxx::builder::basic::kvp(name, val));
    m_read_view_dirty = true;
}

bool Container::getBool(const std::string& name) {

    if (m_read_view_dirty) {
        synchronize();
    }
    
    return m_read_view[name].get_bool();
}


void Container::append(const std::string& name, int32_t val) {
    m_builder.append(bsoncxx::builder::basic::kvp(name, val));
    m_read_view_dirty = true;
}

int32_t Container::getInt32(const std::string& name) {

    if (m_read_view_dirty) {
        synchronize();
    }

    return m_read_view[name].get_int32();
}


void Container::append(const std::string& name, int64_t val) {
  
    bsoncxx::types::b_int64 wrapper;
    wrapper.value = val;

    m_builder.append(bsoncxx::builder::basic::kvp(name, wrapper));
    m_read_view_dirty = true;
}
    
int64_t Container::getInt64(const std::string& name) {
    
    if (m_read_view_dirty) {
        synchronize();
    }

    return m_read_view[name].get_int64();

}


void Container::append(const std::string& name, double val) {
    m_builder.append(bsoncxx::builder::basic::kvp(name, val));
    m_read_view_dirty = true;
}

double Container::getDouble(const std::string& name) {

    if (m_read_view_dirty) {
        synchronize();
    }

    return m_read_view[name].get_double();
}


void Container::append(const std::string& name, const std::string& val) {
    m_builder.append(bsoncxx::builder::basic::kvp(name, val));
    m_read_view_dirty = true;
}

const std::string Container::getString(const std::string& name) {

    if (m_read_view_dirty) {
        synchronize();
    }

    return m_read_view[name].get_utf8().value.to_string();
}

void Container::append(const std::string& name, const uint8_t* data, size_t size) {
    
    bsoncxx::types::b_binary wrapper;
    wrapper.sub_type = bsoncxx::binary_sub_type::k_binary;
    wrapper.size = size;
    wrapper.bytes = data;
    
    m_builder.append(bsoncxx::builder::basic::kvp(name, wrapper));
    m_read_view_dirty = true;
}

const uint8_t* Container::getBinData(const std::string& name, size_t& size) {

    if (m_read_view_dirty) {
        synchronize();
    }

    const bsoncxx::types::b_binary wrapper = m_read_view[name].get_binary();
    size = static_cast<size_t>(wrapper.size);

    return wrapper.bytes;
}


const Container::Ptr Container::getContainer(const std::string& name) {
    
    /*
        Note that we can ONLY append to a bson builder! Hence, when nesting containers
        we need to must create the components individually and then merge them on serialization.
    */

    std::unordered_map<std::string, Container::Ptr>::const_iterator result = m_children.find(name);
    
    if (result != m_children.end()) {
        return result->second;
    }

    Container::Ptr created = std::make_shared<Container>();
    // Need to make a copy of name here as it's only a reference.
    // child_wrappers[name] = created;
    //std::string x = name;
    //std::pair<std::string, MongoBsonMetadataContainer::Ptr> pair = std::make_pair<std::string, MongoBsonMetadataContainer::Ptr>(x, created);
    m_children[name] = created;

    return std::static_pointer_cast<Container>(created);

}

void Container::synchronize() {
    // Simply load in the new tmp object.
    m_read_view = m_builder.view();
    m_read_view_dirty = false;
}

void Container::serialize(bsoncxx::builder::basic::sub_document& builder) {

    builder.append(bsoncxx::builder::concatenate(m_builder.view()));

    for (auto& entry : m_children) {
        
        builder.append(bsoncxx::builder::basic::kvp(entry.first, 
            [&entry](bsoncxx::builder::basic::sub_document sub_document_builder) {
                entry.second->serialize(sub_document_builder);
            }));

    }    

}

}