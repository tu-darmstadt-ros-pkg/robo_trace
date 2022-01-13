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
    Persistor();

    /**
     *
     */
    virtual ~Persistor();

    /**
     *
     */
    virtual void store(const bsoncxx::document::value& element) = 0;

};

}