#pragma once
// Std
#include <string>
// MongoCXX
#include <mongocxx/options/insert.hpp>
// Project
#include "robo_trace/storage/persistor.hpp"


namespace robo_trace::store {

class DirectPersistor final : public Persistor {

public:

    /**
     *
     */
    DirectPersistor(const std::string& database, const std::string& collection);

    /**
     *
     */
    ~DirectPersistor();
    
    /**
     *
     */
    virtual void store(bsoncxx::document::value& element) final override;

private:

    /** */
    mongocxx::options::insert m_insert_option;

};

}