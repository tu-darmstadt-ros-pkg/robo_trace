// Base
#include "robo_trace/storage/result.hpp"
// Std
#include <stdexcept>
// Ros
#include <ros/console.h>


namespace robo_trace {

MessageQueryResultIterator::MessageQueryResultIterator(const std::shared_ptr<mongo::DBClientConnection>& connection, const mongo::Query& query, const std::string& path) {

    /*
        Execute the query and fetch the first element.
    */

    m_result_cursor = connection->query(path, query);
    
    if (m_result_cursor->more()) {
        m_next_element = m_result_cursor->next(); 
    }

}

MessageQueryResultIterator::~MessageQueryResultIterator() {

}

bool MessageQueryResultIterator::isValid() {
    return m_next_element.has_value();
}

bool MessageQueryResultIterator::isCompleted() {
    return !m_result_cursor->more();
}
    
void MessageQueryResultIterator::next() {

    if (!m_next_element.has_value()) {
        return;
    }

    if (m_result_cursor->more()) {
        m_next_element = std::make_optional(m_result_cursor->next());
    } else {
        m_next_element = std::nullopt;
    }

}   

const mongo::BSONObj MessageQueryResultIterator::getMessage() const {

    if (!m_next_element) {
        throw std::runtime_error("Result iterator has no more data.");
    }

    return m_next_element->getField("message").Obj();
}

const mongo::BSONObj MessageQueryResultIterator::getMetadata() const {

    if (!m_next_element) {
        throw std::runtime_error("Result iterator has no more data.");
    }

    return m_next_element->getField("metadata").Obj();
}

}