/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */
#pragma once

// Project
#include "robo_trace/modes/replay/player.hpp"


namespace robo_trace::replay {

class RoboTracePlayer final : public PlayerBase {

public:

    typedef std::shared_ptr<RoboTracePlayer> Ptr;
    typedef std::shared_ptr<const RoboTracePlayer> ConstPtr;

public:

    /**
     *
     */
    RoboTracePlayer(ros::NodeHandle& system_node_handle);

    /**
     *
     */
    ~RoboTracePlayer();

    /**
     *
     */
    bool isCompleted() const;

    /**
     *
     */
    void run();

protected:

    /**
     * 
     */
    virtual void initialize() final override;
    
    /**
     *
     */
    virtual bool isToBePlayedBack(const std::string& topic) const final override;
 
};

}