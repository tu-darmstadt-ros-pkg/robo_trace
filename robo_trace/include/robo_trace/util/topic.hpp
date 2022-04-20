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
#include <vector>


namespace black_box_recorder {
namespace utils {


/**
 * Checks if some topic should be recoreded or not.
 * 
 * @param topic the name of the topic to check.
 * 
 * @returns whether the provided topic is to be subscribed or not.
 */
bool isTopicToBeRecorded(const std::string &topic);

/**
 * Checks whether some topic is currently beeing recorded or not.
 * 
 * @param topic the name of the topic to check.
 * 
 * @returns whether the provided topic is currently beeing recorded or not.s
 */    
bool isTopicBeeingRecorded(const std::string &topic);



/**
 * Check all available topics currently active and subscribes to those
 * that have to be recorded.
 */ 
void onCheckAvailableTopics();


void getAllTopics(std::vector<const std::string> &store);

void getTopicsSubscribedByNode(std::vector<const std::string> &store, const std::string& name);



}
}