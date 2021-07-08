#pragma once

#define ROBO_TRACE_NODE_NAME "robo_trace"

// ########### ROS Logging related definitions ###########

#define ROS_LOGGING_ORCHESTRATOR_NAME "orchestrator"

// ########### ROS Param related definitions ###########

#define ROS_PARAM_PROCESSING_PLUGINS_NAME "processing"

// ########### Dynamic reconfigure related definitions ###########


#define DYNAMIC_RECONFIGURE_CHECK_FOR_TOPICS_PERIOD_NAME "check_for_topics_period"
#define DYNAMIC_RECONFIGURE_CHECK_FOR_TOPICS_PERIOD_DESCRIPTION "The period in seconds in which to check for new topics."
#define DYNAMIC_RECONFIGURE_CHECK_FOR_TOPICS_PERIOD_DEFAULT 0.5
#define DYNAMIC_RECONFIGURE_CHECK_FOR_TOPICS_PERIOD_MIN 0.0
#define DYNAMIC_RECONFIGURE_CHECK_FOR_TOPICS_PERIOD_MAX 60.0

#define DYNAMIC_RECONFIGURE_PAUSE_RECORDING_NAME "pause_recodring"
#define DYNAMIC_RECONFIGURE_PAUSE_RECORDING_DESCRIPTION "Pauses the logging of all data streams in the system."
#define DYNAMIC_RECONFIGURE_PAUSE_RECORDING_DEFAULT false