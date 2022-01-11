#pragma once

// Std
#include <string>
#include <memory>
// MongoDB
#include <bsoncxx/document/view.hpp>
#include <bsoncxx/builder/basic/document.hpp>


namespace robo_trace::store {

class Container {

public:

    typedef std::shared_ptr<Container> Ptr;

    typedef std::shared_ptr<const Container> ConstPtr;

public:

    /**
     *
     */    
    Container();

    /**
     *
     */
    Container(const bsoncxx::document::view& other);

    /**
     *
     */
    ~Container();


    void append(const std::string& name, bool val);

    bool getBool(const std::string& name);


    void append(const std::string& name, int val);
    
    int getInt(const std::string& name);


    void append(const std::string& name, double val);

    double getDouble(const std::string& name);


    void append(const std::string& name, const std::string& val);

    const std::string getString(const std::string& name);


    void append(const std::string& name, const uint8_t* data, size_t size);

    const uint8_t* getBinData(const std::string& name, size_t& size); 


    /**
     * Returns thet metadata container associated to the given key. A 
     * corresponding container will be created if not present.
     * 
     * @param name the key under which the nested container is stored.
     * @return the nested container store.
     */
    const Container::Ptr getContainer(const std::string& name);

     /**
     * 
     */
    void serialize(bsoncxx::builder::basic::sub_document& builder);

private:

    /**
     * 
     */
    void synchronize();

private:

    /** */
    // mongo::BSONObjBuilder m_bson_builder;
    bsoncxx::builder::basic::document m_builder;

    /** */
    bool m_read_view_dirty;
    /** */
    // mongo::BSONObj m_read_object;
    bsoncxx::document::view m_read_view;

    /**  */
    std::unordered_map<std::string, Container::Ptr> m_children;

};

}