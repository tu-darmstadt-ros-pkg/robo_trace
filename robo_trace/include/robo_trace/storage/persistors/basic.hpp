#pragma once
// Std
#include <string>
// MongoCXX
// Project
#include "robo_trace/storage/persistor.hpp"


namespace robo_trace::store {

class BasicPersistor final : public Persistor {

public:

    /**
     *
     */
    BasicPersistor(const std::string database, const std::string collection);

    /**
     *
     */
    ~BasicPersistor();
    
    /**
     *
     */
    virtual void store(const bsoncxx::document::view& element) final override;

private:

    /** */
    const std::string m_database;
    /** */
    const std::string m_collection;

};

}