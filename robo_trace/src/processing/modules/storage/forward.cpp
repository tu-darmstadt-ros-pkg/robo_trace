/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/processing/modules/storage/forward.hpp"
// Std
#include <string>
#include <mutex>
#include <stdexcept>
#ifdef EVALUATION_CAPTURE_COURSE_TIMINIGS
#include <chrono>
#endif
// MongoCXX
#include <bsoncxx/json.hpp>
#include <bsoncxx/stdx/optional.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>


namespace robo_trace::processing {

StorageForwardProcessor::StorageForwardProcessor(const robo_trace::store::Persistor::Ptr& persistor, const robo_trace::store::Container::Ptr& metadata) 
: m_persistor(persistor) {
    //
}

StorageForwardProcessor::~StorageForwardProcessor() {
    //
}

Mode StorageForwardProcessor::getMode() const {
    return Mode::CAPTURE;
}

void StorageForwardProcessor::process(const Context::Ptr& context) {

    bsoncxx::builder::basic::document builder{};
    
    /*
        Serialize the message.
    */
    
    const std::optional<bsoncxx::document::view>& serialized_message_o = context->getBsonMessage();

    if (!serialized_message_o) {
        throw std::runtime_error("Failed persisting message! Writeback stage reached, but not yet serialized.");
    }

    builder.append(bsoncxx::builder::basic::kvp("message", serialized_message_o.value()));

    /*
        Serialize the metadata.
    */

#ifdef EVALUATION_CAPTURE_COURSE_TIMINIGS
    // TODO: We are missing the time needed for serializating of the metadata.
    context->getMetadata()->getContainer("stamps")->append("egress", static_cast<int64_t>(
         std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
    ));
#endif

    builder.append(bsoncxx::builder::basic::kvp("metadata", 
        [&context](bsoncxx::builder::basic::sub_document sub_document_builder) {
            context->getMetadata()->serialize(sub_document_builder);
    }));

    /*
        Push the message to Persistor.
    */

    bsoncxx::document::value entry = builder.extract();
//    ROS_INFO_STREAM(bsoncxx::to_json(entry));
    m_persistor->store(entry);

}



}