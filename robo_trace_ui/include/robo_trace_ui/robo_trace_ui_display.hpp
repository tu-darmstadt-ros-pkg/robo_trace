#pragma once

// Hector Rviz Overlay
#include <hector_rviz_overlay/displays/qml_overlay_display.h>

namespace robo_trace_ui {
 
/**
 * 
 * Refear to the following resource for a reference demo:
 *  -> https://git.sim.informatik.tu-darmstadt.de/hector/hector_rviz_overlay/-/blob/master/hector_rviz_overlay_demo/src/demo_qml_ros_overlay_display.cpp
 * 
 */
class RoboTraceUiDisplay final : public hector_rviz_overlay::QmlOverlayDisplay {
Q_OBJECT

public:

  RoboTraceUiDisplay(); 

  ~RoboTraceUiDisplay();

protected:
    
    virtual QString getPathToQml() final override;

};

}
