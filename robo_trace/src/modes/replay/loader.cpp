// Base
#include "robo_trace/modes/replay/loader.hpp"
// Ros
#include <ros/console.h>
// Project
#include "robo_trace/util/smart_ptr_conversions.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/context.hpp"


namespace robo_trace {


MessageLoader::MessageLoader(const ConnectionOptions::ConstPtr& connector_options, const mongo::Query& query, const std::string& collection_path, std::vector<ProcessingStage::Ptr>& pipeline, ros::CallbackQueueInterface* callback_queue)
: m_connector_options(connector_options), m_query(query), m_collection_path(collection_path), m_processing_pipeline(pipeline), m_scheduling_queue(callback_queue),
  m_query_buffering_batch_size(5), m_deserialization_buffering_threshold(50), m_deserialization_buffering_batch_size(25),
  m_execution_pending(false), m_message_cursor_depleted(false),
  m_message_queue_size(0) {
      
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

void MessageLoader::setQueryBufferingBatchSize(size_t size) {
    
    m_query_buffering_batch_size = size;

    if (m_message_cursor == nullptr) {
        return;
    }

    m_message_cursor->setBatchSize(size);

}

size_t MessageLoader::getQueryBufferUtilization() const {
    return m_query_buffering_batch_size;
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
    
    // The query has not been executed yet.
    if (m_message_cursor == nullptr) {

        // If there is an error, this flag won't be toggled back and this loader terminats.
        m_message_cursor_depleted = true;

        m_connection = ConnectionProvider::getConnection(m_connector_options); 
            
        m_message_cursor = m_connection->query(
            // ns
            m_collection_path, 
            // query
            m_query,
            // nToReturn
            0,
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
    }  
    
    for(size_t message_idx = 0; message_idx < m_deserialization_buffering_batch_size; message_idx++) {
        
        if (!m_message_cursor->more()) {
            break;
        }

        mongo::BSONObj serialized_message = m_message_cursor->next();
        digest(serialized_message);

    }

    // We are done here. 
    m_message_cursor_depleted = !m_message_cursor->more();
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