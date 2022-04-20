/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

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