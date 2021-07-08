// Base
#include "robo_trace_plugin_interface/processing/configurable/descriptor.hpp"
// Std
#include <algorithm>
#include <functional>
// Project
#include "robo_trace_plugin_interface/defnitions.hpp"


namespace robo_trace {

template<typename C>
ConfigurableProcessingStageDescriptor::ConfigurableProcessingStageDescriptor(const ros::NodeHandle& plugin_namespace, const std::string name) 
: ProcessingStageDescriptor(plugin_namespace, name) {

    bool lazy_loading = handle().param<bool>(CONFIGURATION_FIELD_LAZY_LOADING_NAME, CONFIGURATION_FIELD_LAZY_LOADING_DEFAULT);

    std::vector<std::string> variant_names;
    handle().getParam(CONFIGURATION_GROUP_VERSIONS_NAME, variant_names);
    
    for (const std::string& variant_name : variant_names) {
        
        // Should be node_ns/plugin_ns/descriptor_ns/matching/variant_name
        const ros::NodeHandle& variant_matching_ns = ros::NodeHandle(handle(), ros::names::append(CONFIGURATION_GROUP_MATCHING_NAME, variant_name));
        
        uint32_t priority;
        variant_matching_ns.getParam(CONFIGURATION_FIELD_MATCHING_PRIORITY_NAME, priority);

        std::string regex_string;
        variant_matching_ns.getParam(CONFIGURATION_FIELD_MATCHING_REGEX_NAME, regex_string);

        ConfigurationVariant& variant_data = configurations.emplace_back(variant_name, priority, std::regex(regex_string));

        if (lazy_loading) {
            continue;
        }

        const ros::NodeHandle& variant_settings_ns = ros::NodeHandle(handle(), ros::names::append(CONFIGURATION_GROUP_SETTINGS_NAME, variant_name));

        // Assuming there exists a default constructor.
        variant.configuration = std::make_shared<C>();
        getConfiguration(variant.configuration, ns);

    }

    // Sort such that elements are decreasing in priority.
    std::sort(std::begin(configurations), std::end(configurations), [](const ConfigurationVariant& a, const ConfigurationVariant& b) {
        return a.priority > b.priority;
    });

}

template<typename C>
ConfigurableProcessingStageDescriptor::~ConfigurableProcessingStageDescriptor() {
    // Just to make it explicit here.
    configurations.clear();
}   


template<typename C>
const std::shared_ptr<C>& ConfigurableProcessingStageDescriptor::getConfiguration(const std::string& topic) {

    std::vector<ConfigurationVariant>::size_type idx = 0;
    
    // The configuration vector is already sored by priority, so we can just iterate 
    // and get the first match.
    for (; idx < configurations.size(); ++idx) {
        if (std::regex_match(topic, configurations[idx].regex)) {
            break;
        }
    }

    if (idx == configurations.size()) {
        // TODO: Error.
    }

    ConfigurationVariant& variant = configurations[idx];

    // Not jet loaded. Probably in lazy mode.
    if (variant.configuration == std::nullptr_t) {
        // https://docs.ros.org/en/diamondback/api/roscpp/html/namespaceros_1_1names.html
        const ros::NodeHandle& ns = ros::NodeHandle(handle(), ros::names::append(CONFIGURATION_GROUP_SETTINGS_NAME, configuration.name));
        // Assuming there exists a default constructor.
        variant.configuration = std::make_shared<C>();
        getConfiguration(variant.configuration, ns);
    }

    return variant.configuration;
}

}