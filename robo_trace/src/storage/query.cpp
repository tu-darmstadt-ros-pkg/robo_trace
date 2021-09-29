// Base
#include "robo_trace/storage/query.hpp"

namespace robo_trace {


MessageQuery::MessageQuery() = default;

MessageQuery::~MessageQuery() = default;


void MessageQuery::setFieldEquals(const std::string& field, const std::string& value) {
    m_bson_builder << field << value;
}

void MessageQuery::setFieldEquals(const std::string& field, const int value) {
    m_bson_builder << field << value;
}

void MessageQuery::setFieldEquals(const std::string& field, const bool value) {
    m_bson_builder << field << value;
}


void MessageQuery::setFieldLessThan(const std::string& field, const double value) {
    m_bson_builder << field << mongo::LT << value;
}

void MessageQuery::setFieldLessThan(const std::string& field, const int value) {
    m_bson_builder << field << mongo::LT << value;
}


void MessageQuery::setFieldEqualLessThan(const std::string& field, const double value) {
    m_bson_builder << field << mongo::LTE << value;
}

void MessageQuery::setFieldEqualLessThan(const std::string& field, const int value) {
    m_bson_builder << field << mongo::LTE << value;
}


void MessageQuery::setFieldGreaterThan(const std::string& field, const double value) {
    m_bson_builder << field << mongo::GT << value;
}

void MessageQuery::setFieldGreaterThan(const std::string& field, const int value) {
    m_bson_builder << field << mongo::GT << value;
}


void MessageQuery::setFieldEqualGreaterThan(const std::string& field, const double value) {
    m_bson_builder << field << mongo::GTE << value;
}

void MessageQuery::setFieldEqualGreaterThan(const std::string& field, const int value) {
    m_bson_builder << field << mongo::GTE << value;
}


void MessageQuery::setFieldInRangeExclusive(const std::string& field, const double lower_bound, const double upper_bound) {
    m_bson_builder << field << mongo::GT << lower_bound << mongo::LT << upper_bound;
}

void MessageQuery::setFieldInRangeExclusive(const std::string& field, const int lower_bound, const int upper_bound) {
    m_bson_builder << field << mongo::GT << lower_bound << mongo::LT << upper_bound;
}


void MessageQuery::setFieldInRangeInclusive(const std::string& field, const double lower_bound, const double upper_bound) {
    m_bson_builder << field << mongo::GTE << lower_bound << mongo::LTE << upper_bound;
}

void MessageQuery::setFieldInRangeInclusive(const std::string& field, const int lower_bound, const int upper_bound) {
    m_bson_builder << field << mongo::GTE << lower_bound << mongo::LTE << upper_bound;
}

const MessageQuery::Ptr MessageQuery::getQuery(const std::string& name) {

    std::unordered_map<std::string, MessageQuery::Ptr>::const_iterator result = m_sub_queries.find(name);
    
    if (result != m_sub_queries.end()) {
        return result->second;
    }

    MessageQuery::Ptr created = std::make_shared<MessageQuery>();
    m_sub_queries[name] = created;

    return created;
}

void MessageQuery::serialize(mongo::BSONObjBuilder& builder) {

    builder.appendElements(m_bson_builder.obj());

    for (auto& entry : m_sub_queries) {
        
        // Builder appending to current byte array  
        mongo::BufBuilder& sub_object_buffer_start = builder.subobjStart(entry.first);
        mongo::BSONObjBuilder sub_object_builder(sub_object_buffer_start);

        entry.second->serialize(sub_object_builder);
        // Sub builder finished.
        sub_object_builder.done();

    }

}



}