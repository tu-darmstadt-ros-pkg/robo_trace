// Base
#include "robo_trace_core/utils/wrapped_bson.hpp"


namespace robo_trace {

using mongo::BSONObj;
using mongo::BSONObjBuilder;

WrappedBSON::WrappedBSON() 
: BSONObj(), m_builder(new BSONObjBuilder()) {
    //
}

WrappedBSON::WrappedBSON(const WrappedBSON& other) 
: BSONObj(), m_builder(other.m_builder) {
    update();
}

WrappedBSON::WrappedBSON(const BSONObj& other) 
: BSONObj(), m_builder(new BSONObjBuilder()) {
    m_builder->appendElements(other);
    update();
}

WrappedBSON::WrappedBSON(const std::string& json) 
: BSONObj(), m_builder(new BSONObjBuilder()) {
    m_builder->appendElements(mongo::fromjson(json.c_str()));
    update();
}

void WrappedBSON::update() {
    BSONObj::operator=(m_builder->asTempObj());
}

}