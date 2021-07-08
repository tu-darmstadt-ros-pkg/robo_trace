#pragma once
// Std
#include <string>
#include <memory>


namespace robo_trace {

/**
 * TODO: Design: Maybe move to persistance folder.
 */
class MetadataContainer {

public:

    typedef std::shared_ptr<MetadataContainer> Ptr;

    typedef std::shared_ptr<const MetadataContainer> ConstPtr;

public:

    
    MetadataContainer() = default;

    virtual ~MetadataContainer() = default;

 
    virtual void append(const std::string& name, const bool val) = 0;

    virtual bool getBool(const std::string& name) = 0;


    virtual void append(const std::string& name, const int val) = 0;
    
    virtual int getInt(const std::string& name) = 0;


    virtual void append(const std::string& name, const double val) = 0;

    virtual double getDouble(const std::string& name) = 0;


    virtual void append(const std::string& name, const std::string val) = 0;

    virtual const std::string getString(const std::string& name) = 0;


    virtual void append(const std::string& name, const void* data, size_t size) = 0;

    virtual const void* getBinData(const std::string& name, size_t& size) = 0; 


    /**
     * Returns thet metadata container associated to the given key. A 
     * corresponding container will be created if not present.
     * 
     * @param name the key under which the nested container is stored.
     * @return the nested container store.
     */
    virtual const MetadataContainer::Ptr getContainer(const std::string& name) = 0;

};

}