// Base
#include "robo_trace/storage/persistors/batch.hpp"
// Std
#ifdef EVALUATION_CAPTURE_WRITEBACK_TIME
#include <chrono>
#endif
// Ros
#include <ros/ros.h>
// MongoCXX
#include <bsoncxx/json.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
#include <mongocxx/write_concern.hpp>
#include <mongocxx/result/bulk_write.hpp>
// Project
#include "robo_trace/util/smart_ptr_conversions.hpp"
#include "robo_trace/storage/connector.hpp"


namespace robo_trace::store {

BatchPersistor::BatchPersistor(const std::string& database, const std::string& collection, ros::CallbackQueueInterface* callback_queue)
: BatchPersistor(database, collection, callback_queue, PERSISTOR_DEFAULT_BATCH_SIZE) {
    //
}

BatchPersistor::BatchPersistor(const std::string& database, const std::string& collection, ros::CallbackQueueInterface* callback_queue, uint32_t buffer_size) 
: Persistor(database, collection), m_buffer_size(buffer_size), m_scheduling_queue(callback_queue) {
    
#ifdef PERSISTOR_BYPASS_DOCUMENT_VALIDATION
    m_write_options.bypass_document_validation(false);
#endif


    mongocxx::write_concern wc_majority{};
    wc_majority.acknowledge_level(mongocxx::write_concern::level::k_majority);
    m_write_options.write_concern(wc_majority);

    Persistor::setup();

    // Create the initial Task.
    m_next_task = std::make_shared<BatchPersistor::Task>(m_write_options);

}

BatchPersistor::~BatchPersistor() = default;

uint32_t BatchPersistor::getBufferSize() const {
    return m_buffer_size;
}

void BatchPersistor::setBufferSize(const uint32_t size) {
    m_buffer_size = size;
}

void BatchPersistor::flush() {
    schedule<false>();
}

void BatchPersistor::store(bsoncxx::document::value& element) {
    
    m_next_task->add(element);

    if (m_next_task->getSize() < m_buffer_size) {
        return;
    }

    schedule<true>();

}


template<bool enable_threshold_check = true>
void BatchPersistor::schedule() {

    std::lock_guard<std::mutex> guard(m_scheduling_mutex);

    uint32_t utilization = m_next_task->getSize();

    if (enable_threshold_check) {
        if (utilization < m_buffer_size) {
            return;
        }
    } else {
        // No need to do anything.
        if (utilization == 0) {
            return;
        }
    }

    BatchPersistor::Task::Ptr m_pending = m_next_task;

    m_next_task = std::make_shared<BatchPersistor::Task>(m_write_options);
    
    // Update database and collection before commencing the upload. Might has been changed after a flush.
    m_pending->setDatabase(m_database);
    m_pending->setCollection(m_collection);
    // Submit to the job queue.
    m_scheduling_queue->addCallback(to_boost_ptr(m_pending));
  
}



BatchPersistor::Task::Task(const mongocxx::options::bulk_write& write_options, const std::string& database, const std::string& collection)
: m_write_options(write_options), m_database(database), m_collection(collection) {
    // 
}

BatchPersistor::Task::Task(const mongocxx::options::bulk_write& write_options)
: m_write_options(write_options) {
    // 
}

BatchPersistor::Task::~Task() = default;

bool BatchPersistor::Task::isCompleted() {
    return m_completed;
}

size_t BatchPersistor::Task::getSize() {
    return models.size();
}

std::string BatchPersistor::Task::getDabase() const {
    return m_database;
}

void BatchPersistor::Task::setDatabase(const std::string& database) {
    m_database = database;
}

std::string BatchPersistor::Task::getCollection() const {
    return m_collection;
}

void BatchPersistor::Task::setCollection(const std::string& collection) {
    m_collection = collection;
}

void BatchPersistor::Task::add(bsoncxx::document::value& value) { 
    
    //   mongocxx::model::insert_one model{value.view()};
   
    models.emplace_back(value.view());
    values.push_back(std::move(value));
}

bool BatchPersistor::Task::ready() {
    return true;
}

ros::CallbackInterface::CallResult BatchPersistor::Task::call() {

    auto client = robo_trace::store::Connector::instance().getClient();
    auto database = (*client)[m_database];

#ifdef EVALUATION_CAPTURE_WRITEBACK_TIME
    const int64_t writeback_time_start = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); 
#endif

    bsoncxx::stdx::optional<mongocxx::result::bulk_write> result = database[m_collection].bulk_write(models, m_write_options);

#ifdef EVALUATION_CAPTURE_WRITEBACK_TIME
    const int64_t writeback_time_end = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); 
#endif

#if defined(PERSISTOR_VALIDATE_MESSAGE_WRITEBACK) || defined(EVALUATION_CAPTURE_WRITEBACK_TIME) 
    if (!result || result.value().inserted_count() != models.size()) {
        throw std::runtime_error("Failed storing messages for collection " + m_collection + " (Affected: " + std::to_string(result.value().inserted_count()) + "/" + std::to_string(models.size()) + ")!");
    }
#endif

#ifdef EVALUATION_CAPTURE_WRITEBACK_TIME

    auto evaluation_data_builder = bsoncxx::builder::basic::document{};
    evaluation_data_builder.append(bsoncxx::builder::basic::kvp("start", writeback_time_start));
    evaluation_data_builder.append(bsoncxx::builder::basic::kvp("end", writeback_time_end));
    evaluation_data_builder.append(bsoncxx::builder::basic::kvp("topic", m_collection));
    evaluation_data_builder.append(bsoncxx::builder::basic::kvp("count", static_cast<int32_t>(models.size())));

    database["evaluation"].insert_one(std::move(evaluation_data_builder.extract()));

/*
Attempt 1: Failed due to some missing implementation in MongoDB....

    auto evaluation_data_builder = bsoncxx::builder::basic::document{};
    
     // Append the time measurements
    evaluation_data_builder.append(bsoncxx::builder::basic::kvp("stamps", 
        [writeback_time_start, writeback_time_end](bsoncxx::builder::basic::sub_document sub_document) {
            sub_document.append(bsoncxx::builder::basic::kvp("start", writeback_time_start));
            sub_document.append(bsoncxx::builder::basic::kvp("end", writeback_time_end));
    }));
    
    // WTF. RETURNS NOTHING!
    const std::map<std::size_t, bsoncxx::document::element>& ids = result.value().upserted_ids();

    // Append the ids
    evaluation_data_builder.append(bsoncxx::builder::basic::kvp("ids", 
        [&ids](bsoncxx::builder::basic::sub_array sub_array) {
            for (const auto& id : ids) {
                sub_array.append(id.second.get_oid().value);
            }
    }));
*/

#endif

    // Free the buffers.
    models.clear();
    values.clear();

    m_completed = true;

}

}