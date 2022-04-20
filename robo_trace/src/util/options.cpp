/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "robo_trace/util/options.hpp"
// Std
#include <stdexcept>
#include <sstream>
// Boost
#include "boost/program_options.hpp"


namespace po = boost::program_options;

namespace robo_trace::util {


void Options::load(const std::vector<Options::Ptr>& containers, ros::NodeHandle& node_handle, int argc, char** argv) {

    /*
        Load all the parameters that there are into the .
    */

    po::options_description description("Allowed options");
    
    description.add_options()
        ("help,h", "produce help message");
        
    for (const Options::Ptr& container : containers) {
        container->setup(description);
    }

    po::positional_options_description p;
    po::variables_map vm;

    try  {
        po::store(po::command_line_parser(argc, argv).options(description).positional(p).run(), vm);
    } catch (const boost::program_options::invalid_command_line_syntax& e) {
        throw std::runtime_error(e.what());
    } catch (const boost::program_options::unknown_option& e) {
        throw std::runtime_error(e.what());
    }

    if (vm.count("help")) {
        std::cout << description << std::endl;
        exit(0);
    }

    /*
        Load in the parameters from ROS.
    */

    for (const Options::Ptr& container : containers) {
        container->load(node_handle);
    }

    /*
        Load in the parameters from the CMD line. This may override ROS parameter definitions.
    */

    for (const Options::Ptr& container : containers) {
        container->load(vm);
    }

    /*
        Sanity check if the prameters make sense.
    */

    for (const Options::Ptr& container : containers) {
        container->validate();
    }

}


Options::Options() = default;

Options::~Options() = default;


void Options::validate() {
    //
}


}