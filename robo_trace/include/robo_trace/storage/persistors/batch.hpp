#pragma once
// Std
#include <utility>
#include <memory>
#include <mutex>
#include <vector>
// MongoCXX
#include <mongocxx/options/bulk_write.hpp>
#include <mongocxx/model/insert_one.hpp>
#include <mongocxx/options/gridfs/bucket.hpp>
// Ros
#include <ros/callback_queue_interface.h>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/storage/persistor.hpp"


namespace robo_trace::store {

class BatchPersistor final : public Persistor {

private:

    class Task final : public ros::CallbackInterface {
        
    public:

        /** */
        typedef std::shared_ptr<Task> Ptr;
        /** */
        typedef std::shared_ptr<const Task> ConstPtr;

    public:

        /**
         * 
         */
        Task(const mongocxx::options::bulk_write& write_options);

        /**
         * 
         */
        Task(const mongocxx::options::bulk_write& write_options, const std::string& database, const std::string& collection);

        /**
         * 
         */
        ~Task();

        /**
         * 
         */
        bool isCompleted();

        /**
         * 
         */
        size_t getSize();

        /**
         * 
         */
        std::string getDabase() const;

        /**
         * 
         */
        void setDatabase(const std::string& database);
        
        /**
         * 
         */
        std::string getCollection() const;

        /**
         * 
         */
        void setCollection(const std::string& collection);

        /**
         * 
         */
        void add(bsoncxx::document::value& value);

    private:

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
        volatile bool m_completed;

        /** */
        std::string m_database;
        /** */
        std::string m_collection;
        /** */
        const mongocxx::options::bulk_write& m_write_options; 

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
    uint32_t getBufferSize() const;

    /**
     * 
     */
    void setBufferSize(const uint32_t size);

    /**
     *
     */
    virtual void flush() final override; 

    /**
     *
     */
    virtual void store(bsoncxx::document::value& element) final override;

private:

    /**
     * 
     */
    template<bool enable_threshold_check = true>
    void schedule();

private:

    /** */
    uint32_t m_buffer_size;

    /** */
    mongocxx::options::bulk_write m_write_options;

    /** */
    std::mutex m_scheduling_mutex;
    /** */
    ros::CallbackQueueInterface* m_scheduling_queue;

    /** */
    Task::Ptr m_next_task;

};

}