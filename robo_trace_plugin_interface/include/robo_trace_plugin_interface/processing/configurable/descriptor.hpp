#pragma once

// Std
#include <memory>
#include <vector>
#include <string>
#include <regex>
// Project
#include "robo_trace_plugin_interface/processing/descriptor.hpp"
#include "robo_trace_plugin_interface/processing/configurable/stage.hpp"


namespace robo_trace {

/**
 * 
 * 
 */
template<class C>
class ConfigurableProcessingStageDescriptor : public ProcessingStageDescriptor {

private:

    /**
     * Summarizes a configuration variant.
     */
    struct ConfigurationVariant {
        
        ConfigurationVariant(const std::string name, const uint32_t priority, const std::regex regex) 
        : name(name), priority(priority), regex(regex) {
            // 
        }

        /** The name of this configuration variant as defined in the settings. */
        const std::string name;

        /** The priority for matching this variant. */
        const uint32_t priority;

        /** The regex to check if topic apply to this variant */
        const std::regex regex;

        /** The a pointer to the configuration. May be a null_ptr.  */
        std::shared_ptr<C> configuration;

    };

public:

    /**
     * TODO
     */
    ConfigurableProcessingStageDescriptor(const ros::NodeHandle& plugin_namespace, const std::string name);

    /**
     * TODO
     */
    virtual ~ConfigurableProcessingStageDescriptor();

protected:

    /**
     * Provides a pointer to the configuration instance to use for the
     * provided topic name.
     * 
     * @param topic the name of topic for which to load the settings.
     * 
     * @return a pointer to the configuration instance.
     */
    const std::shared_ptr<C>& getConfiguration(const std::string& topic);

    /**
     * Loads the configuration from the namespace, provided through the 
     * ROS NodeHandle, into the provided configuration instance. 
     * 
     * TODO: This is nice and game for the FORWARD case, as we can simply
     * load from the parameter server, but how do we load stuff  in the other 
     * cases, where are basically the emitter of everything?
     * 
     * @param configuration a pointer to the configuration instance to load the settings into. 
     * @param handle the NodeHandle encapsualting the namespace from which to load the settings.
     */
    virtual void getConfiguration(const std::shared_ptr<C>& configuration, const ros::NodeHandle& handle) = 0; 

private:

    const std::vector<ConfigurationVariant> configurations;

};

}