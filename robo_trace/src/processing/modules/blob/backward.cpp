/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Project
#include "robo_trace/processing/stage/blob/backward.hpp"
// Std
#include <mutex>
#include <sstream>


namespace robo_trace {

BlobDecouplingBackwardStage::BlobDecouplingBackwardStage(const std::shared_ptr<mongo::DBClientConnection>& connection, const std::string& database) 
: m_database_name(database), 
  m_connection(connection),
  m_grid_filesystem(new mongo::GridFS(*m_connection, m_database_name)) {
   //
}

BlobDecouplingBackwardStage::~BlobDecouplingBackwardStage() = default;

ProcessingMode BlobDecouplingBackwardStage::getMode() const {
    return ProcessingMode::CAPTURE;
}

void BlobDecouplingBackwardStage::process(const ProcessingContext::Ptr& context) {

    if (!context->isSerialized()) {
        throw std::runtime_error("Message has reached database writeback stage, but is not serialized yet.");
    }

    const mongo::BSONObj& serialized_message = context->getSerializedMessage().value();
    
    mongo::BSONObjBuilder builder;
    couple(builder, serialized_message);
        
    context->setSerializedMessage(builder.obj());
   
}

void BlobDecouplingBackwardStage::couple(mongo::BSONObjBuilder& builder_decoupled, const mongo::BSONObj& serialized) {
     
    for(mongo::BSONObj::iterator element_iterator = serialized.begin(); element_iterator.more(); ) { 
        
        const mongo::BSONElement element = element_iterator.next();
        const std::string element_name(element.fieldName());

        switch (element.type()) {
            // Load BinData from GFS
            case mongo::BSONType::jstOID: {

                const mongo::OID gfs_object_id = element.__oid();   
                const mongo::BSONObj gfs_object_query = BSON("_id" << gfs_object_id);                         
                const mongo::GridFile gfc_object_file = m_grid_filesystem->findFile(gfs_object_query);

                std::stringstream stream(std::ios::out | std::ios::binary);
                gfc_object_file.write(stream);

                const std::string binary_string = stream.str();

                builder_decoupled.appendBinData(
                    // name
                    element_name,
                    // length 
                    binary_string.length(),  
                    // type
                    mongo::BinDataType::BinDataGeneral, 
                    // data
                    binary_string.data()
                ); 
                break;
            }
            // 
            case mongo::BSONType::Array:
            case mongo::BSONType::Object: {
                
                mongo::BufBuilder& sub_buff_builder = builder_decoupled.subobjStart(element_name);
                mongo::BSONObjBuilder sub_obj_builder(sub_buff_builder);
                
                const mongo::BSONObj& object = element.Obj();
                couple(sub_obj_builder, object);
                
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