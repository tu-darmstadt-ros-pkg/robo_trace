#pragma once
// Std
#include <mutex>
#include <vector>
#include <atomic>
// MongoCXX
#include <mongocxx/options/bulk_write.hpp>
#include <mongocxx/model/insert_one.hpp>
// Ros
#include <ros/callback_queue_interface.h>
// Project
#include "robo_trace/util/align.hpp"
#include "robo_trace/storage/persistor.hpp"


namespace robo_trace::store {

class BatchPersistor final : public Persistor, public ros::CallbackInterface, public std::enable_shared_from_this<BatchPersistor> {

private:

    /**
     * 
     */
    struct PingPongData {
        /** */
        std::vector<mongocxx::model::insert_one> models;
        /** Buffers the document value object, hence hindering release of the underling buffers. */
        std::vector<bsoncxx::document::value> values;
    };
    

public:

    /**
     * 
     */
    BatchPersistor(const std::string& database, const std::string& collection, ros::CallbackQueueInterface* callback_queue);

    /**
     * 
     */
    BatchPersistor(const std::string& database, const std::string& collection, ros::CallbackQueueInterface* callback_queue, uint32_t buffer_size);

    /**
     * 
     */
    virtual ~BatchPersistor();
    
    /**
     *
     */
    virtual void store(bsoncxx::document::value& element) final override;

private:

    /**
     * 
     */
    void schedule();

    /**
     *
     */
    virtual bool ready() final override;

    /**
     *
     */
    virtual ros::CallbackInterface::CallResult call() final override;

private:

    /** */
    const uint32_t m_buffer_size;

    /** */
    mongocxx::options::bulk_write m_insert_option;
    /** */
    ros::CallbackQueueInterface* m_scheduling_queue;

    /** */
    uint8_t m_buffer_index;
    /** */
    std::atomic<bool> m_upload_pending;

    /** */
    PingPongData m_buffer[2];

};

}