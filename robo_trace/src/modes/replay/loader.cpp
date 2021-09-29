// Base
#include "robo_trace/modes/replay/loader.hpp"
// Ros
#include <ros/console.h>
// Project
#include "robo_trace/util/smart_ptr_conversions.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/context.hpp"

#include <fstream>
#include <iostream>

namespace robo_trace {


MessageLoader::MessageLoader(const ConnectionOptions::ConstPtr& connector_options, const std::optional<mongo::BSONObj>& structure_query, const std::string& collection_path, std::vector<ProcessingStage::Ptr>& pipeline, ros::CallbackQueueInterface* callback_queue, const std::optional<double>& time_start, const std::optional<double>& time_end)
: m_connector_options(connector_options), m_query_structure(structure_query), m_collection_path(collection_path),
  m_processing_pipeline(pipeline), m_scheduling_queue(callback_queue), m_query_time_start(time_start), m_query_time_end(time_end),
  m_query_buffering_batch_size(5), m_deserialization_buffering_threshold(75), m_deserialization_buffering_batch_size(5),
  m_execution_pending(false), m_message_cursor_depleted(false),
  m_message_queue_size(0), m_time_last_batch_end(0) {
      
      // We can not schedule the first invocation of the loader here, as a context switch might
      // call the loader before the construction is completed!

}
   
MessageLoader::~MessageLoader() {
    //
}

bool MessageLoader::isCompleted() const {
    // Note that this might return true even if there are no more messages. The 
    // difficulty lies in the fact that the mongo query may only be accessed by
    // one thread alone! 
    return m_message_queue_size == 0 && m_message_cursor_depleted;
} 

bool MessageLoader::isBuffering() const {
    return m_execution_pending;
}

size_t MessageLoader::getDeserializationBufferUtilization() const {
    return m_message_queue_size;
}

size_t MessageLoader::getQueryBufferingBatchSize() const {
    return m_query_buffering_batch_size;
}

void MessageLoader::setQueryBufferingBatchSize(size_t size) {
    m_query_buffering_batch_size = size;
}

size_t MessageLoader::getDeserializationBufferingThreshold() const {
    return m_deserialization_buffering_threshold;
}

void MessageLoader::setDeserializationBufferingThreshold(size_t threshold) {
    m_deserialization_buffering_threshold = threshold;
}
    
size_t MessageLoader::getDeserializationBufferingBatchSize() const {
    return m_deserialization_buffering_batch_size;
}
    
void MessageLoader::setDeserializationBufferingBatchSize(size_t size) {
    // TODO: Should we reschedule here?
    m_deserialization_buffering_batch_size = size;
}
    
const std::optional<std::pair<double, ros_babel_fish::BabelFishMessage::ConstPtr>> MessageLoader::next() {

    if (m_message_queue_size == 0) {
        return std::nullopt;
    }

    const std::optional<std::pair<double, ros_babel_fish::BabelFishMessage::ConstPtr>> message = m_message_queue.try_pop();

    --m_message_queue_size;
    // Maybe reschedule if the buffer is depleted.
    schedule();

    return message;
}
   
void MessageLoader::schedule() {
   
    // Secure this method by a lock.
    std::lock_guard<std::mutex> guard(m_scheduling_mutex);

    if (m_deserialization_buffering_threshold < m_message_queue_size || m_execution_pending || m_message_cursor_depleted) {
        return;
    }

    m_execution_pending = true;
    m_scheduling_queue->addCallback(to_boost_ptr(shared_from_this()));
    
}

bool MessageLoader::ready() {
    return true;
}

ros::CallbackInterface::CallResult MessageLoader::call() {
    
    bool is_first_execution = false;

    // The query has not been executed yet.
    if (m_connection == nullptr) {

        // If there is an error, this flag won't be toggled back and this loader terminats.
        m_message_cursor_depleted = true;
        is_first_execution = true;

        // Establish a non-thread save connection.
        m_connection = ConnectionProvider::getConnection(m_connector_options); 

    }  

    mongo::BSONObjBuilder query_time_constrains;

    // Consider start time constrains.
    if (is_first_execution && m_query_time_start) {
        query_time_constrains << "metadata.time" << mongo::GTE << m_query_time_start.value();
    } else {
        query_time_constrains << "metadata.time" << mongo::GT << m_time_last_batch_end;
    }

    // Consider end time constrains
    if (m_query_time_end) {
        query_time_constrains << "metadata.time" << mongo::LT << m_query_time_end.value();
    }

    mongo::BSONObjBuilder query_builder;
    query_builder.appendElements(query_time_constrains.obj());

    if (m_query_structure) {
        query_builder.appendElements(m_query_structure.value());
    }
    //

    const mongo::BSONObj structure_query = query_builder.done();
    //std::cout << "Query: " << structure_query.jsonString() << std::endl;

    mongo::Query query(structure_query);
    query.sort("metadata.time", 1);

/*
    mongo::BSONObjBuilder dirty_hacks;
    dirty_hacks.appendElements(query.obj);
    dirty_hacks << "allowDiskUse" << true;
    query.obj = dirty_hacks.done();
    std::cout << "Query compiled: " << query.obj.jsonString() << std::endl;
*/

    std::unique_ptr<mongo::DBClientCursor> message_cursor = m_connection->query(
        // ns
        m_collection_path, 
        // query
        query,
        // nToReturn
        m_deserialization_buffering_batch_size,
        // nToSkip
        0,
        // fieldsToReturn (Note: That batch mode does not work here, since it 
        // requires the same thread to always fetch the data.)
        0,
        // queryOptions
        0,
        // batchSize
        m_query_buffering_batch_size
    );
    
    size_t message_idx = 0;
    
    while(message_cursor->more()) {

        mongo::BSONObj serialized_message = message_cursor->next();
        
        std::ofstream out("output.txt");
        out << serialized_message.jsonString();
        out.close();

        digest(serialized_message);

        m_time_last_batch_end = serialized_message["metadata"]["time"]._numberDouble();
        ++message_idx;

    }

    // We are done here. 
    m_message_cursor_depleted = (message_idx < m_deserialization_buffering_batch_size);
    m_execution_pending = false;
    
    // Check reschedule if in the meantime someone consumed from the buffer.
    schedule();

    return ros::CallbackInterface::CallResult::Success;

}

void MessageLoader::digest(const mongo::BSONObj& serialized_message) {

    /*
        Reconstruct metadata wrapper.
    */

    const mongo::BSONObj& metadata_serialized = serialized_message["metadata"].Obj();
    const DataContainer::Ptr metadata_container = std::make_shared<DataContainer>(metadata_serialized); 

    /*
        Handle the message depending on the type.
    */

    switch (serialized_message["type"].numberInt()) {
            
        // Is reconfiguration request.
        case 0: {
            
            /*
                Pass the configuration request to the stages.
                TODO: Remove - legacy stuff.
            */

            break;
        }
        // Is a message. 
        case 1: {

            /*
                Construct the processing context.
            */

            const mongo::BSONObj& message_serialized = serialized_message["message"].Obj();
            
            const ProcessingContext::Ptr context = std::make_shared<ProcessingContext>(metadata_container);
            context->setSerializedMessage(message_serialized);

            /*
                Process  the message.
            */
            
            for (const ProcessingStage::Ptr& processing_stage : m_processing_pipeline) {
                processing_stage->process(context);
            }
            
            /*
                Push messages to sink.
            */

            const std::optional<ros_babel_fish::BabelFishMessage::ConstPtr>& o_message_deserialized = context->getUnserializedMessage();

            if (!o_message_deserialized) {
                throw std::runtime_error("Message has reached final stage, but is not deserialized yet.");
            }

            const ros_babel_fish::BabelFishMessage::ConstPtr& message_deserialized = o_message_deserialized.value();
            const double message_ingress = metadata_container->getDouble("time");
            
            std::pair<double, ros_babel_fish::BabelFishMessage::ConstPtr> entry = std::make_pair(message_ingress, message_deserialized);
            m_message_queue.push(std::move(entry)); 

            ++m_message_queue_size;

            break;
        }
        default: 
            throw std::runtime_error("Invalid element type.");

    }
    
}

}