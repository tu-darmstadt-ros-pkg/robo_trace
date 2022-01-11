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