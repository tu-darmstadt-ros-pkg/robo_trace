// Base
#include "robo_trace_ui/robo_trace_ui_display.hpp"
// Plugin Lib
#include <pluginlib/class_list_macros.h>


namespace robo_trace_ui {

RoboTraceUiDisplay::RoboTraceUiDisplay() 
: hector_rviz_overlay::QmlOverlayDisplay() {
    //
}

RoboTraceUiDisplay::~RoboTraceUiDisplay() = default;

QString RoboTraceUiDisplay::getPathToQml() {
    return "package://robo_trace_ui/media/UiOverlay.qml";
}

}

PLUGINLIB_EXPORT_CLASS(robo_trace_ui::RoboTraceUiDisplay, rviz::Display)