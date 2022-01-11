#pragma once

namespace robo_trace::processing::sink {

class Persistor {

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
    virtual void store(const bsoncxx::document::view& element) = 0;

};

}