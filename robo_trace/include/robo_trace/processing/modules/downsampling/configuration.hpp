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
#include <string>
#include <memory>
#include <regex>


namespace robo_trace::processing {

class DownsamplingMode final {

public:

    enum MatchingTarget {
        /** */
        TOPIC,
        /** */
        TYPE
    };

    enum Strategy {
        /** */
        TIME,
        /** */
        COUNT
    };

public:

    typedef std::shared_ptr<DownsamplingMode> Ptr;
    typedef std::shared_ptr<const DownsamplingMode> ConstPtr;

public:

    /**
     * 
     */
    DownsamplingMode(const std::string& name, const DownsamplingMode::MatchingTarget target, const int priority, const std::regex regex, const DownsamplingMode::Strategy strategy, const double value);

    /**
     * 
     */
    ~DownsamplingMode();

    /**
     * 
     */
    const std::string& getName() const;

    /**
     *
     */
    const DownsamplingMode::MatchingTarget getMatchingTarget() const;

    /**
     * 
     */
    const int getPriority() const;

    /**
     * 
     */
    const std::regex getRegex() const;

    /**
     *
     */
    const DownsamplingMode::Strategy getDownsamplingStrategy() const;

    /**
     *
     */
    double getValue() const;

private:

    /** */
    const std::string m_name;

    /** */
    const DownsamplingMode::MatchingTarget m_matching_target;
    /** */
    const int m_matching_priority;
    /** */
    const std::regex m_matching_regex;

    /** */
    const DownsamplingMode::Strategy m_downsampling_strategy;
    /** */
    const double m_downsampling_value;

};   

}