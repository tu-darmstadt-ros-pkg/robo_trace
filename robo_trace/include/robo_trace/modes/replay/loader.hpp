#pragma once

// Std
#include <functional>
#include <memory>
#include <atomic>
#include <utility>
#include <mutex>
#include <optional>
// MongoDB
#include "robo_trace/config.h"
#include <mongo/client/dbclient.h>
// Ros
#include <ros/callback_queue_interface.h>
// BabelFish
#include <ros_babel_fish/babel_fish_message.h>
// Project
#include "robo_trace/util/ts_queue.hpp"
#include "robo_trace/storage/container.hpp"
#include "robo_trace/storage/options.hpp"
#include "robo_trace/processing/stage/stage.hpp"


namespace robo_trace {

class MessageLoader final : public ros::CallbackInterface, public std::enable_shared_from_this<MessageLoader> {

public:

    typedef std::shared_ptr<MessageLoader> Ptr;
    typedef std::shared_ptr<const MessageLoader> ConstPtr;

public:

    /**
     *
     */
    MessageLoader(const ConnectionOptions::ConstPtr& connection_options, 
                  const std::optional<mongo::BSONObj>& structure_query, 
                  const std::string& collection_path, 
                  std::vector<ProcessingStage::Ptr>& pipeline, 
                  ros::CallbackQueueInterface* callback_queue,
                  const std::optional<double>& time_start,
                  const std::optional<double>& time_end);

    /**
     *
     */
    ~MessageLoader();

    /**
     *
     */
    bool isCompleted() const; 

    /**
     *
     */
    bool isBuffering() const;
    
    /** 
     *
     */
    size_t getDeserializationBufferUtilization() const;

    /** 
     *
     */
    size_t getQueryBufferingBatchSize() const;

    /**
     *
     */
    void setQueryBufferingBatchSize(size_t size);
  
    /**
     *
     */
    size_t getDeserializationBufferingThreshold() const;
    
    /**
     *
     */
    void setDeserializationBufferingThreshold(size_t threshold);
    
    /**
     *
     */
    size_t getDeserializationBufferingBatchSize() const;
    
    /**
     *
     */
    void setDeserializationBufferingBatchSize(size_t size);

    /**
     *
     */
    void schedule();

    /** 
     * Returns the next message to be played back and the corresponding
     * playback time. This removes the returned message from the buffer
     * and with the next invocation, the next message is returned.
     */
    const std::optional<std::pair<double, ros_babel_fish::BabelFishMessage::ConstPtr>> next();
    
private:

    /**
     *
     */
    virtual bool ready() final override;

    /**
     *
     */
    virtual ros::CallbackInterface::CallResult call() final override;

    /**
     *
     */
    void digest(const mongo::BSONObj& serialized_message);

private:
    
    /** */
    const std::string m_collection_path;

    /** */
    const ConnectionOptions::ConstPtr m_connector_options;
    /** */
    const std::vector<ProcessingStage::Ptr> m_processing_pipeline;
    
    /** */
    const std::optional<double>& m_query_time_start;
    /** */
    const std::optional<double>& m_query_time_end;
     /** */
    const std::optional<mongo::BSONObj>& m_query_structure; 

    /** */
    std::mutex m_scheduling_mutex;
    /** */
    ros::CallbackQueueInterface* m_scheduling_queue;

    /** */
    size_t m_query_buffering_batch_size;
    /** */
    size_t m_deserialization_buffering_threshold;
    /** How many elements to deserialize at max. */
    size_t m_deserialization_buffering_batch_size;
    
    /** */
    std::atomic<bool> m_execution_pending;
    /** */
    std::atomic<bool> m_message_cursor_depleted;
    /** */
    double m_time_last_batch_end;
   
    /** */
    std::shared_ptr<mongo::DBClientConnection> m_connection;
        
    /** */
    std::atomic<int> m_message_queue_size;
    /** */
    ts_queue<std::pair<double, ros_babel_fish::BabelFishMessage::ConstPtr>> m_message_queue;


};

}