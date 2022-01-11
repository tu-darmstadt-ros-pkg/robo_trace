// Base
#include "robo_trace/processing/modules/downsampling//configuration.hpp"


namespace robo_trace::processing {

DownsamplingMode::DownsamplingMode(const std::string& name, const DownsamplingMode::MatchingTarget target, const int priority, const std::regex regex, const DownsamplingMode::Strategy strategy, const double value) 
: m_name(name),
  m_matching_target(target),
  m_matching_priority(priority),
  m_matching_regex(regex),
  m_downsampling_strategy(strategy),
  m_downsampling_value(value) {
      //
}

DownsamplingMode::~DownsamplingMode() = default;

const std::string& DownsamplingMode::getName() const {
    return m_name;
}

const DownsamplingMode::MatchingTarget DownsamplingMode::getMatchingTarget() const {
    return m_matching_target;
}

const int DownsamplingMode::getPriority() const {
    return m_matching_priority;
}

const std::regex DownsamplingMode::getRegex() const {
    return m_matching_regex;
}

const DownsamplingMode::Strategy DownsamplingMode::getDownsamplingStrategy() const {
    return m_downsampling_strategy;
}

double DownsamplingMode::getValue() const {
    return m_downsampling_value;
}


}