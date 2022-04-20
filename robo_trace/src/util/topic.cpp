/*
 * Copyright (c) 2022 Fachgebiet Simulation, Systemoptimierung und Robotik, TU Darmstadt.
 *
 * This file is part of RoboTrace
 * (see https://github.com/tu-darmstadt-ros-pkg/robo_trace) 
 * and is governed by a BSD-style license 
 * that can be found in the LICENSE file.
 */

// Base
#include "black_box_recorder/orchestrator.hpp"
// Boost
#include <boost/foreach.hpp>
// Ros
#include <ros/ros.h>
#include <ros/network.h>
#include <ros/xmlrpc_manager.h>
// XmlRpc
#include <xmlrpcpp/XmlRpc.h>


namespace black_box_recorder {
namespace utils {

void getAllTopics(std::vector<const std::string> &store) {
    /*
        This lookup is inspired by how rosbag does it:
         -> https://github.com/ros/ros_comm/blob/kinetic-devel/tools/rosbag/src/recorder.cpp#L567        
    */

    std::vector<const std::string> topic_names;

    if(!ros::master::getTopics(topic_info)) {
        return;
    }

    BOOST_FOREACH(ros::master::TopicInfo const& info, topic_infos) {
        store.push_back(info.name);
    }

}

void getTopicsSubscribedByNode(std::vector<const std::string> &store, const std::string& node) {
    /*
        This lookup is inspired by how rosbag does it:
         -> https://github.com/ros/ros_comm/blob/kinetic-devel/tools/rosbag/src/recorder.cpp#L567
    */

    XmlRpc::XmlRpcValue node_address_lookup_request;
    node_address_lookup_request[0] = ros::this_node::getName();
    node_address_lookup_request[1] = node;

    XmlRpc::XmlRpcValue node_address_lookup_response;
    XmlRpc::XmlRpcValue node_address_lookup_payload;

    // https://docs.ros.org/en/diamondback/api/roscpp/html/namespaceros_1_1master.html#a6148ee923ef1602d2093daff82573043
    if(!ros::master::execute("lookupNode", node_address_lookup_request, node_address_lookup_response, node_address_lookup_payload, true)) {
        return;
    }

    std::string peer_host;
    uint32_t peer_port;

    if(!ros::network::splitURI(static_cast<std::string>(response[2]), peer_host, peer_port)) {
        ROS_ERROR("Bad xml-rpc URI trying to inspect node at: [%s]", static_cast<std::string>(resp[2]).c_str());
        return;
    }

    XmlRpc::XmlRpcValue node_get_subscriptions_request;
    node_get_subscriptions_request[0] = ros::this_node::getName();

    XmlRpc::XmlRpcValue node_get_subscriptions_response;

    XmlRpc::XmlRpcClient client(peer_host.c_str(), peer_port, "/");
    client.execute("getSubscriptions", node_get_subscriptions_request, node_get_subscriptions_response);

    if(client.isFault() || !node_get_subscriptions_response.valid() || node_get_subscriptions_response.size() == 0 || static_cast<int>(node_get_subscriptions_response[0]) != 1) {
        ROS_ERROR("Node at: [%s] failed to return subscriptions.", static_cast<std::string>(resp[2]).c_str());
        return;
    }

    for(int i = 0; i < node_get_subscriptions_response[2].size(), ++i) {
        store.push_back(node_get_subscriptions_response[2][i][0]);
    }

}

}
}