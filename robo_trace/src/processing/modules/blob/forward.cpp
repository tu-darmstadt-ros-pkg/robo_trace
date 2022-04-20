/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Project
#include "robo_trace/processing/stage/blob/forward.hpp"
// Std
#include <mutex>


namespace robo_trace {

BlobDecouplingForwardStage::BlobDecouplingForwardStage(const std::shared_ptr<mongo::DBClientConnection>& connection, const std::string& database) 
: m_database_name(database), 
  m_connection(connection),
  m_grid_filesystem(new mongo::GridFS(*m_connection, m_database_name)) {
   //
}

BlobDecouplingForwardStage::~BlobDecouplingForwardStage() = default;

ProcessingMode BlobDecouplingForwardStage::getMode() const {
    return ProcessingMode::CAPTURE;
}

void BlobDecouplingForwardStage::process(const ProcessingContext::Ptr& context) {


    if (!context->isSerialized()) {
        throw std::runtime_error("Message has reached database writeback stage, but is not serialized yet.");
    }

    const mongo::BSONObj& serialized_message = context->getSerializedMessage().value();
    
    mongo::BSONObjBuilder builder;
    decouple(builder, serialized_message);
        
    context->setSerializedMessage(builder.obj());
   
}


void BlobDecouplingForwardStage::decouple(mongo::BSONObjBuilder& builder_decoupled, const mongo::BSONObj& serialized) {
     
    for(mongo::BSONObj::iterator element_iterator = serialized.begin(); element_iterator.more(); ) { 
        
        const mongo::BSONElement element = element_iterator.next();
        const std::string element_name(element.fieldName());

        switch (element.type()) {
            // Outsource BinData to GFS
            case mongo::BSONType::BinData: {

                int blob_data_size = 0;
                const char* blob_data = element.binData(blob_data_size);

                mongo::OID blod_file_id;
                mongo::BSONObj blob_file_object = m_grid_filesystem->storeFile(blob_data, blob_data_size, blod_file_id.toString());

                mongo::OID blob_id;
                blob_file_object["_id"].Val(blob_id);
    
                builder_decoupled.append(element_name, blob_id);    
                break;
            }
            // 
            case mongo::BSONType::Array:
            case mongo::BSONType::Object: {
                
                mongo::BufBuilder& sub_buff_builder = builder_decoupled.subobjStart(element_name);
                mongo::BSONObjBuilder sub_obj_builder(sub_buff_builder);
                
                const mongo::BSONObj& object = element.Obj();
                decouple(sub_obj_builder, object);
                
                sub_obj_builder.done();
                break;
            }
            default: {
                builder_decoupled.append(element);
                break;
            }

        } 

    }

}



}