/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
#pragma once

// ROS
#include <ros/ros.h>


namespace robo_trace::replay {

class TimeManager {

public:

    /**
     *
     */
    TimeManager(ros::NodeHandle& node_handle);

    /**
     *
     */
    ~TimeManager();

    /**
     *
     */
    bool isHorizonReached();

    /**
     *
     */
    double getTimeScale() const;

    /**
     *
     */
    void setTimeScale(double time_scale);

    /**
     *
     */
    const ros::Time& getStartTimeReal() const;

    /**
     *
     */
    void setStartTimeReal(const ros::Time& time);

    /**
     *
     */
    const ros::Time& getStartTimeTranslated() const;
    /**
     *
     */
    void setStartTimeTranslated(const ros::Time & time); 

    /**
     *
     */
    double getTimePublishFrequency() const;

    /**
     *
     */
    void setTimePublishFrequency(double publish_frequency);
    
    /**
     *
     */
    const ros::Time& getCurrentTime() const;

    /**
     *
     */
    void setCurrentTime(const ros::Time& time);

    /**
     *
     */
    const ros::Time& getExecutionHorizon() const;

    /**
     * Set the horizon that the clock will run to.
     */
    void setExecutionHorizon(const ros::Time& horizon);

    /**
     *
     */
    void advance(const ros::Duration& duration);

    /**
     * Run the clock for AT MOST duration. If horizon has
     * been reached this function returns immediately.
     */
    void run(const ros::WallDuration& duration);

    /**
     *
     */
    void stalled(const ros::WallDuration& duration);

    /**
     *
     */
    void step();


private:

    /** */
    ros::NodeHandle m_node_handle;
    
    /** */
    double m_time_scale;

    /** */
    ros::Time m_current_time; 
    /** */
    ros::Time m_start_time_real;
    /** */
    ros::Time m_start_time_translated;

    /** */
    ros::Time m_horizon;
    /** */
    ros::WallTime m_horizon_wc;

    /** */
    bool m_publication_enabled;
    /** */
    double m_publication_frequency;
    /** */
    ros::WallDuration m_publication_wall_step;
    /** */
    ros::WallTime m_publication_next_time;
    /** */
    ros::Publisher m_publication_publisher;
     
};

}