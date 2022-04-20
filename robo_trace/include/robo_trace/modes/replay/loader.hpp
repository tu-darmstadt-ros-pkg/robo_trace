/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
#pragma once

// Std
#include <functional>
#include <memory>
#include <atomic>
#include <utility>
#include <mutex>
#include <optional>
// MongoCXX
#include <bsoncxx/document/value.hpp>
#include <bsoncxx/document/view.hpp>
// Ros
#include <ros/callback_queue_interface.h>
// BabelFish
#include <ros_babel_fish/babel_fish_message.h>
// Project
#include "robo_trace/util/ts_queue.hpp"
#include "robo_trace/storage/container.hpp"
#include "robo_trace/processing/processor.hpp"


namespace robo_trace::replay {

class MessageLoader final : public ros::CallbackInterface, public std::enable_shared_from_this<MessageLoader> {

public:

    typedef std::shared_ptr<MessageLoader> Ptr;
    typedef std::shared_ptr<const MessageLoader> ConstPtr;

public:

    /**
     *
     */
    MessageLoader(const std::string& collection, 
                  const std::string& database,
                  const std::vector<robo_trace::processing::Processor::Ptr>& pipeline, 
                  ros::CallbackQueueInterface* callback_queue,
                  const std::optional<double>& time_start,
                  const std::optional<double>& time_end,
                  const std::optional<bsoncxx::document::value>& structure_query);

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
    bool isValid() const;

    /**
     *
     */
    bool isLoading() const;

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
    void reset(const double time);

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
    void digest(const bsoncxx::document::view& serialized_message);

private:
    
    /** */
    const std::string m_collection;
    /** */
    const std::string m_database;

    /** */
    const std::optional<double> m_query_time_start;
    /** */
    const std::optional<double> m_query_time_end;
     /** */
    const std::optional<bsoncxx::document::value> m_query_structure; 
   
    /** */
    robo_trace::store::StreamHandler::Ptr m_stream_handler;
    /** */
    const std::vector<robo_trace::processing::Processor::Ptr> m_processing_pipeline;
   
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
    std::atomic<bool> m_flush_pending;
    /** */
    std::atomic<bool> m_terminal;
    /** */
    std::atomic<double> m_time_last_batch_end;
       
    /** */
    std::atomic<int> m_message_queue_size;
    /** */
    ts_queue<std::pair<double, ros_babel_fish::BabelFishMessage::ConstPtr>> m_message_queue;


};

}