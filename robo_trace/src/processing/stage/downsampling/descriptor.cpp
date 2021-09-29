// Base
#include "robo_trace/processing/stage/downsampling/descriptor.hpp"
// Std
#include <stdexcept>
// Project
#include "robo_trace/parameters.hpp"
#include "robo_trace/storage/connector.hpp"
#include "robo_trace/processing/stage/downsampling/forward_count.hpp"
#include "robo_trace/processing/stage/downsampling/forward_time.hpp"


namespace robo_trace {

DownsamplingStageDescriptor::DownsamplingStageDescriptor(const ros::NodeHandle& stages_namespace) 
: ProcessingStageDescriptor(stages_namespace, "downsampling") {
      
    if (!m_handle.hasParam("targets")) {
        return;
    }

    XmlRpc::XmlRpcValue models;
    m_handle.getParam("targets", models);

    for(XmlRpc::XmlRpcValue::ValueStruct::const_iterator it_model = models.begin(); it_model != models.end(); ++it_model) { 

        const std::string& model_name = it_model->first;
        const XmlRpc::XmlRpcValue& model =  it_model->second;
       
        if (model.getType() != XmlRpc::XmlRpcValue::Type::TypeStruct) {
            throw std::runtime_error("Failed loading downsampling model. Unexpected syntax.");
        }

        const std::string target_string = model["matching"];
        DownsamplingMode::MatchingTarget target;

        if (target_string == "topic") {
            target = DownsamplingMode::MatchingTarget::TOPIC;
        } else if (target_string == "type") {
            target = DownsamplingMode::MatchingTarget::TYPE;
        } else {
            throw std::runtime_error("Failed loading downsampling model. Unknown matching stratgey.");
        }

        const std::string strategy_string = model["strategy"];
        DownsamplingMode::Strategy strategy;

        if (strategy_string == "time") {
            strategy = DownsamplingMode::Strategy::TIME;
        } else if (strategy_string == "count") {
            strategy = DownsamplingMode::Strategy::COUNT;
        } else {
            throw std::runtime_error("Failed loading downsampling model. Unknown downsampling stratgey.");
        }

        const int priority = model["priority"];
        const double value = model["value"];
        const std::string regex = model["regex"];
        
        DownsamplingMode::Ptr mode_descriptor = std::make_shared<DownsamplingMode>(
            // name
            model_name, 
            // target 
            target,
            // priority
            priority, 
            // regex
            std::regex(regex), 
            // strategy
            strategy,
            // value
            value
        );

        m_downsampling_modes.push_back(mode_descriptor);
        
    }

    std::sort(m_downsampling_modes.begin(), m_downsampling_modes.end(), 
        [](const DownsamplingMode::Ptr& one, const DownsamplingMode::Ptr& two) -> bool { 
            return one->getPriority() > two->getPriority(); 
        }
    );

}

DownsamplingStageDescriptor::~DownsamplingStageDescriptor() = default;
    
bool DownsamplingStageDescriptor::isModeSupported(const ProcessingMode mode) const  {
    return mode == ProcessingMode::CAPTURE;
}

std::optional<ProcessingStage::Ptr> DownsamplingStageDescriptor::getStage(const DataContainer::Ptr& summary, const ProcessingMode mode) {

    if (mode != ProcessingMode::CAPTURE) {
        throw std::runtime_error("Cant serve the requested mode.");
    }

    size_t mode_idx = 0;
    
    // The configuration vector is already sored by priority, so we can just iterate 
    // and get the first match.
    for (; mode_idx < m_downsampling_modes.size(); ++mode_idx) {

        const DownsamplingMode::Ptr& downsampling_mode = m_downsampling_modes[mode_idx];
        std::string key;

        // TODO: Can we get this mapping a little bit nicer? Maybe by using a map?
        switch (downsampling_mode->getMatchingTarget()) {
            case DownsamplingMode::MatchingTarget::TOPIC:
                key = "topic";
                break;
            case DownsamplingMode::MatchingTarget::TYPE:
                key = "message_type";
                break;
        }  

        if (std::regex_match(summary->getString(key), downsampling_mode->getRegex())) {
            break;
        }
  
    }

    if (mode_idx == m_downsampling_modes.size()) {
        return {};
    }

    const DownsamplingMode::Ptr& downsampling_mode = m_downsampling_modes[mode_idx];

    switch(downsampling_mode->getDownsamplingStrategy()) {
        case DownsamplingMode::Strategy::COUNT : {
            return std::make_shared<CountDownsamplingForwardStage>(
                // value - encodes how many messages to "leave out"
                static_cast<uint32_t>(downsampling_mode->getValue())
            );
        }
        case DownsamplingMode::Strategy::TIME : {

            ros::Duration period(downsampling_mode->getValue()); 

            return std::make_shared<TimeDownsamplingForwardStage>(
                // value - encodes how period in which to capture
                period
            );
        }
        default: {
            throw std::runtime_error("Unsupported downsampling strategy selected.");
        }
            
        
    }
}


}