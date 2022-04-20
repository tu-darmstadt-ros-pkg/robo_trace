# RoboTrace
This repository contains the source files for RoboTrace.

## Dependencies And Setup
RoboTrace is beeing developed primarily for ROS `melodic` middelware. The choice for this older verison of ROS, is due to a simulation, internal to the SIM Group @ TU Darmstadt, only beeing available for this corresponding distribution of ROS. In any case, besides this framework, the following packages for it must be installed:

 - [Ros Babel Fish](https://github.com/StefanFabian/ros_babel_fish)
 - [PluginLib](http://wiki.ros.org/pluginlib)
 - [DDynamicReconfigure](https://github.com/pal-robotics/ddynamic_reconfigure)

Additionally the following libraries must be installed on the target platform:

 - [Boost](https://www.boost.org/), explicitly the `program_options` and `regex` modules.
 - [MongoDB C++ drivers](https://www.mongodb.com/docs/drivers/cxx/) 

If everything is setup correctly, the project may be build, using the standard ROS buildtools.  

## Running
Before running RoboTrace, make sure the MongoDB server is up and running. Furthermore as for now, a few configuration parameters must be set through the ROS parameter server. Refear to the `/cfg` directory for examples on how such configuration may look like. You could bootstrap a ROS environment and load these configurations in by a launch file that looks as those in the `/launch` folder. 

Having everything setup, recording can then be commened by `rosrun robo_trace record -a` for capturing all nodes. A regex max also be specified by `-e` and a single topic may be specified by `-topic`.

## Replaying
TODO: Documentation
