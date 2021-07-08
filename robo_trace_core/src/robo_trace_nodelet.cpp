// Std
#include <memory>
// Nodelet
#include <nodelet/nodelet.h>
// PluginLib
#include <pluginlib/class_list_macros.h>
// Project
#include "robo_trace_core/orchestrator.hpp"


namespace robo_trace {

class RoboTraceRecorderNodelet : public nodelet::Nodelet {

    void onInit() override {

        ros::NodeHandle& node_handle = getNodeHandle();
        ros::NodeHandle& node_handle_private = getPrivateNodeHandle();

        /*
            Initialize the orchestrator (kick off everything).
        */

        m_orchestrator = std::make_shared<RoboTraceOrchestrator>();
        m_orchestrator->initialize(node_handle, node_handle_private);

    }

    std::shared_ptr<RoboTraceOrchestrator> m_orchestrator;

};

}

PLUGINLIB_EXPORT_CLASS(robo_trace::RoboTraceRecorderNodelet, nodelet::Nodelet);