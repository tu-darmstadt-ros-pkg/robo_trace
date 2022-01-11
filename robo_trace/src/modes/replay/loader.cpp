// Base
#include "robo_trace/modes/replay/loader.hpp"
// MongoCXX
#include <bsoncxx/types.hpp>
#include <bsoncxx/json.hpp>
#include <bsoncxx/builder/basic/document.hpp>
#include <bsoncxx/builder/basic/kvp.hpp>
#include <mongocxx/options/find.hpp>
#include <mongocxx/client.hpp>
#include <mongocxx/database.hpp>
#include <mongocxx/collection.hpp>
// Ros
#include <ros/console.h>
// Project
#include "robo_trace/util/smart_ptr_conversions.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/context.hpp"


namespace robo_trace::replay {

MessageLoader::MessageLoader(const std::string& collection, const std::string& database, const std::vector<robo_trace::processing::Processor::Ptr>& pipeline, ros::CallbackQueueInterface* callback_queue, const std::optional<double>& time_start, const std::optional<double>& time_end, const std::optional<bsoncxx::document::value>& structure_query)
: m_collection(collection), m_database(database), m_processing_pipeline(pipeline), m_scheduling_queue(callback_queue),
  m_query_time_start(time_start), m_query_time_end(time_end), m_query_structure(structure_query),  
  m_query_buffering_batch_size(5), m_deserialization_buffering_threshold(75), m_deserialization_buffering_batch_size(5),
  m_execution_pending(false), m_flush_pending(false), m_terminal(false),
  m_message_queue_size(0), m_time_last_batch_end(-1) {
      
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
    return !m_flush_pending && !m_execution_pending && m_message_queue_size == 0 && m_terminal;
} 

bool MessageLoader::isValid() const {
    return !m_flush_pending && (m_message_queue_size > 0);
}

bool MessageLoader::isLoading() const {
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

    if (m_flush_pending || m_message_queue_size == 0) {
        return std::nullopt;
    }

    const std::optional<std::pair<double, ros_babel_fish::BabelFishMessage::ConstPtr>> message = m_message_queue.try_pop();
    
    // Might be empty if a flush happened in the meantime.
    if (message) {
        --m_message_queue_size;
    }
   
    // Maybe reschedule if the buffer is depleted.
    schedule();

    return message;
}
   
void MessageLoader::reset(const double time) {

    m_flush_pending = true;  
    m_time_last_batch_end = time;

    schedule();

}

void MessageLoader::schedule() {
   
    // Secure this method by a lock.
    std::lock_guard<std::mutex> guard(m_scheduling_mutex);

    if (m_execution_pending) {
        return;
    }

    if (!m_flush_pending && (m_deserialization_buffering_threshold < m_message_queue_size || m_terminal)) {
        return;
    }

    m_execution_pending = true;
    m_scheduling_queue->addCallback(to_boost_ptr(shared_from_this()));
    
}

bool MessageLoader::ready() {
    return true;
}

ros::CallbackInterface::CallResult MessageLoader::call() {
    
    const double time_last_batch_end = m_time_last_batch_end.load();
    
    bsoncxx::builder::basic::document query_builder{};
   
    if (m_flush_pending) {

        // Clear any already loaded messages.
        m_message_queue.clear();
        m_message_queue_size = 0;

        m_terminal = true;
        m_flush_pending = false;


        query_builder.append(bsoncxx::builder::basic::kvp("metadata.time", 
            [time_last_batch_end](bsoncxx::builder::basic::sub_document sub_document_builder) {
                sub_document_builder.append(bsoncxx::builder::basic::kvp("$gte", time_last_batch_end));
            }
        ));

        // Query from the re-rentry point onwards.
        //query_time_constrains << "metadata.time" << mongo::GTE << m_time_last_batch_end.load();

    } else {
        
        const bool is_first_execution = time_last_batch_end == -1;

        // Consider start time constrains.
        if (is_first_execution && m_query_time_start) {

            const double query_time_start = m_query_time_start.value();

            query_builder.append(bsoncxx::builder::basic::kvp("metadata.time", 
                [query_time_start](bsoncxx::builder::basic::sub_document sub_document_builder) {
                    sub_document_builder.append(bsoncxx::builder::basic::kvp("$gte", query_time_start));
                }
            ));
            // query_time_constrains << "metadata.time" << mongo::GTE << m_query_time_start.value();
        } else {
            query_builder.append(bsoncxx::builder::basic::kvp("metadata.time", 
                [time_last_batch_end](bsoncxx::builder::basic::sub_document sub_document_builder) {
                    sub_document_builder.append(bsoncxx::builder::basic::kvp("$gt", time_last_batch_end));
                }
            ));
            // query_time_constrains << "metadata.time" << mongo::GT << m_time_last_batch_end.load();
        }

    }

    // Consider end time constrains
    // Note: If the end time is greater than the start time, the cursor will be empty and the loader is flagged as depleated.
    if (m_query_time_end) {
        
        const double query_time_end = m_query_time_end.value();;

        query_builder.append(bsoncxx::builder::basic::kvp("metadata.time", 
            [query_time_end](bsoncxx::builder::basic::sub_document sub_document_builder) {
                sub_document_builder.append(bsoncxx::builder::basic::kvp("$lt", query_time_end));
            }
        ));
        //query_time_constrains << "metadata.time" << mongo::LT << m_query_time_end.value();
    }
    
    //mongo::BSONObjBuilder query_builder;
    //query_builder.appendElements(query_time_constrains.obj());

    if (m_query_structure) {
        query_builder.append(bsoncxx::builder::concatenate(m_query_structure.value().view()));
    }
   
    //const mongo::BSONObj structure_query = query_builder.done();
    //std::cout << "Query: " << structure_query.jsonString() << std::endl;
    //const bsoncxx::document::value structure_query = m_query_structure.extract();
    
    mongocxx::options::find options_query;
    options_query.sort(bsoncxx::builder::basic::make_document(bsoncxx::builder::basic::kvp("metadata.time", 1)));
    // Sets the number of documents to fetch per batch from the database.
    options_query.batch_size(m_query_buffering_batch_size);
    // Limits the number of elements to return per query
    options_query.limit(m_deserialization_buffering_batch_size);
    // If the query is to big to be processed in main memory, we'll allow for
    // processing to also utilize the hard disk.
    options_query.allow_disk_use(true);

    //mongo::Query query(structure_query);
    //query.sort("metadata.time", 1);
    
/*
    mongo::BSONObjBuilder dirty_hacks;
    dirty_hacks.appendElements(query.obj);
    dirty_hacks << "allowDiskUse" << true;
    query.obj = dirty_hacks.done();
    std::cout << "Query compiled: " << query.obj.jsonString() << std::endl;
*/  
    
    mongocxx::pool::entry client = robo_trace::store::Connector::instance().getClient();
    // TODO: Database
    //mongocxx::client& dc = *client;
    //mongocxx::database& dx = dc.database(m_database);
    mongocxx::cursor result = (*client)[m_database][m_collection].find(
        // filter
        query_builder.view(),
        // options
        options_query
    );

/*
    std::unique_ptr<mongo::DBClientCursor> message_cursor = m_connection->query(
        // ns
        m_collection_path, 
        // query
        query,
        // nToReturn
        m_deserialization_buffering_batch_size,
        // nToSkip
        0,
        // fieldsToReturn
        0,
        // queryOptions
        0,
        // batchSize
        m_query_buffering_batch_size
    );
*/
    size_t message_idx = 0;
    
    for(mongocxx::cursor::iterator iterator = result.begin(); iterator != result.end(); ++iterator) {

    //while(message_cursor->more()) {
       
        //mongo::BSONObj serialized_message = message_cursor->next();
        const bsoncxx::document::view& serialized_message = *iterator;
        digest(serialized_message);

        // ROS_INFO_STREAM(bsoncxx::to_json(serialized_message, bsoncxx::ExtendedJsonMode::k_relaxed));

        // If there has been a flush, the field "m_time_last_batch_end" will hold
        // the time of re-entry. Hence, we should avoid overwriting it.
        if (m_flush_pending) {
            break;
        }

        m_time_last_batch_end = serialized_message["metadata"]["time"].get_double();
        ++message_idx;

    }

    // We are done here. 
    m_terminal = !m_flush_pending && (message_idx < m_deserialization_buffering_batch_size);
    m_execution_pending = false;
    
    // Check reschedule if in the meantime someone consumed from the buffer.
    schedule();

    return ros::CallbackInterface::CallResult::Success;

}

void MessageLoader::digest(const bsoncxx::document::view& serialized_message) {

    /*
        Reconstruct metadata wrapper.
    */

    const bsoncxx::document::view metadata_serialized = serialized_message["metadata"].get_document().view();
    const robo_trace::store::Container::Ptr metadata_container = std::make_shared<robo_trace::store::Container>(metadata_serialized); 

    /*
        Construct the processing context.
    */

    const bsoncxx::document::view message_serialized = serialized_message["message"].get_document().view();
    
    const robo_trace::processing::Context::Ptr context = std::make_shared<robo_trace::processing::Context>(metadata_container);
    context->setSerializedMessage(message_serialized);

    /*
        Process  the message.
    */
    
    for (const robo_trace::processing::Processor::Ptr& processing_stage : m_processing_pipeline) {
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
 
}

}