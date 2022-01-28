// Base
#include "robo_trace/storage/persistors/batch.hpp"
// Ros
#include <ros/ros.h>
// MongoCXX
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/result/bulk_write.hpp>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/util/smart_ptr_conversions.hpp"
#include "robo_trace/storage/connector.hpp"


namespace robo_trace::store {

BatchPersistor::BatchPersistor(const std::string& database, const std::string& collection, ros::CallbackQueueInterface* callback_queue)
: BatchPersistor(database, collection, callback_queue, PERSISTOR_DEFAULT_BATCH_SIZE) {
    //
}

BatchPersistor::BatchPersistor(const std::string& database, const std::string& collection, ros::CallbackQueueInterface* callback_queue, uint32_t buffer_size) 
: Persistor(database, collection), m_buffer_size(buffer_size), m_scheduling_queue(callback_queue), 
  m_buffer_index(0), m_upload_pending(false) {
    
#ifdef PERSISTOR_BYPASS_DOCUMENT_VALIDATION
    m_insert_option.bypass_document_validation(true);
#endif

    Persistor::setup();

}

BatchPersistor::~BatchPersistor() = default;

void BatchPersistor::schedule() {

    PingPongData& data = m_buffer[m_buffer_index];

    if (data.models.size() < m_buffer_size) {
        return;
    }

    if (m_upload_pending) {
        ROS_INFO_STREAM("Persistor for " << m_collection << " is overfull by " << std::to_string(data.models.size() - m_buffer_size) << " elements and still pending.");
        return;
    }
   
    m_upload_pending = true;
    m_buffer_index = 1 - m_buffer_index;

    m_scheduling_queue->addCallback(to_boost_ptr(shared_from_this()));
        
}


void BatchPersistor::store(bsoncxx::document::value& element) {

    mongocxx::model::insert_one model{element.view()};

    PingPongData& data = m_buffer[m_buffer_index];
    data.models.push_back(std::move(model));
    data.values.push_back(std::move(element));

    // Check if we maybe need to start uploading our data now.
    schedule();

}

bool BatchPersistor::ready() {
    return true;
}

ros::CallbackInterface::CallResult BatchPersistor::call() {

    // The index of the buffer to upload now.
    const uint8_t target_buffer_idx = 1 - m_buffer_index;
    PingPongData& data = m_buffer[target_buffer_idx];

    auto client = robo_trace::store::Connector::instance().getClient();
    auto collection = (*client)[m_database][m_collection];
    
    bsoncxx::stdx::optional<mongocxx::result::bulk_write> result = collection.bulk_write(data.models, m_insert_option);
   
#ifdef PERSISTOR_VALIDATE_MESSAGE_WRITEBACK
    if (!result || result.value().result().inserted_count() != data.models.size()) {
        throw std::runtime_error("Failed storing messages for collection " + m_collection + " (Affected: " + data.models.size() + ")!");
    }
#endif
   
    // Free the buffers.
    data.models.clear();
    data.values.clear();

    m_upload_pending = false;

}

}