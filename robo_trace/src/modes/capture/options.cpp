// Base
#include "robo_trace/modes/capture/options.hpp"
// Std
#include <sstream>
// Boost
#include "boost/program_options.hpp"


namespace po = boost::program_options;

namespace robo_trace::capture {


Options::Options() 
: m_capture_all(false),
  m_capture_node(false),
  m_capture_topics_by_regex(false),
  m_capture_exclude_by_regex(false),
  m_capture_subscriber_queue_size(1),
  m_capture_topic_check_period(0.2) {

};

Options::~Options() = default;


void Options::load(const ros::NodeHandle& node_handle) {

    ros::NodeHandle capture_namespace(node_handle, "capture");
   
    capture_namespace.getParam("all", m_capture_all);
    capture_namespace.getParam("regex", m_capture_topics_by_regex);
    capture_namespace.getParam("topics", m_capture_topics);

    m_capture_node = capture_namespace.hasParam("node");
    capture_namespace.getParam("node", m_capture_node_name);
    
    if (capture_namespace.hasParam("exclude")) {
       
        std::string capture_exclude_regex;
        capture_namespace.getParam("exclude", capture_exclude_regex);

        m_capture_exclude_by_regex = true;
        m_capture_exclude_regex = capture_exclude_regex;
    
    }
  
    capture_namespace.getParam("limit", m_capture_limit_per_topic);
    capture_namespace.getParam("topic_list_update_period", m_capture_topic_check_period);
    capture_namespace.getParam("queue", m_capture_subscriber_queue_size);
    
}

void Options::setup(po::options_description& description) {

    description.add_options()
        ("all,a", "Record all topics")
        ("regex,e", "Match topics using regular expressions")
        ("exclude,x", po::value<std::string>(), "Exclude topics matching regular expressions")
        ("publish,p", "Publish a msg when the record begin")
        ("limit,l", po::value<int>()->default_value(0), "Only record NUM messages on each topic")
        ("topic", po::value< std::vector<std::string>>(), "topic to record")
        ("duration", po::value<std::string>(), "Record a bag of maximum duration in seconds, unless 'm', or 'h' is appended.")
        ("node", po::value<std::string>(), "Record all topics subscribed to by a specific node.")
        ("tcpnodelay", "Use the TCP_NODELAY transport hint when subscribing to topics.")
        ("udp", "Use the UDP transport hint when subscribing to topics.")
        ("topic_update_period", po::value<double>(), "Check in with the master for new topics every NUM seconds.")
        ("queue", po::value<int>(), "use an outgoing queue of size SIZE");

}

void Options::load(po::variables_map& options) {

    if (options.count("all")) {
        m_capture_all = true;
    }
    
    if (options.count("node")) {
        m_capture_node = true;
        m_capture_node_name = options["node"].as<std::string>();
    }

    if (options.count("regex")) {
        m_capture_topics_by_regex = true;
    }

    if (options.count("topic")) {
      
        std::vector<std::string> bags = options["topic"].as<std::vector<std::string>>();
        std::sort(bags.begin(), bags.end());
        bags.erase(std::unique(bags.begin(), bags.end()), bags.end());
        
        for (std::vector<std::string>::iterator i = bags.begin(); i != bags.end(); i++) {
            m_capture_topics.push_back(*i);
        }

    }

    if (options.count("exclude")) {
        m_capture_exclude_by_regex = true;
        m_capture_exclude_regex = options["exclude"].as<std::string>();
    }

    if (options.count("limit")) {
        m_capture_limit_per_topic = options["limit"].as<int>();
    }

    if (options.count("duration")) {
        
        std::string duration_str = options["duration"].as<std::string>();

        double duration;
        double multiplier = 1.0;
        std::string unit("");

        std::istringstream iss(duration_str);

        if ((iss >> duration).fail()) {
            throw std::runtime_error("Duration must start with a floating point number.");
        }

        if ((!iss.eof() && ((iss >> unit).fail()))) {
            throw std::runtime_error("Duration unit must be s, m, or h");
        }

        if (unit == std::string("")) {
            multiplier = 1.0;
        } else if (unit == std::string("s")) {
            multiplier = 1.0;
        } else if (unit == std::string("m")) {
            multiplier = 60.0;
        } else if (unit == std::string("h")) {
            multiplier = 3600.0;
        } else {
            throw std::runtime_error("Duration unit must be s, m, or h");
        }
        
        m_capture_duration = ros::Duration(duration * multiplier);

        if (m_capture_duration <= ros::Duration(0)) {
            throw std::runtime_error("Duration must be positive.");
        }

    }    
    
    if (options.count("tcpnodelay")) {
        m_capture_subscriber_transport_hints.tcpNoDelay();
    }

    if (options.count("udp")) {
        m_capture_subscriber_transport_hints.udp();
    }

    if (options.count("new_topic_update_period")) {
        m_capture_topic_check_period = options["new_topic_update_period"].as<double>();
    }

    if (options.count("queue")) {
        m_capture_subscriber_queue_size = options["queue"].as<int>();
    }

}   

void Options::validate() {

    if (!m_capture_all && m_capture_topics.size() == 0 && m_capture_node_name == "") {
        std::cout << "Not capturing anything." << std::endl; exit(0);
    }

    if (m_capture_topic_check_period <= 0.0) {
        std::cout << "Capture list update period must be greater than 0." << std::endl; exit(0);
    }

    if (m_capture_subscriber_queue_size < 1) {
        std::cout << "Subscriber queue must have at least a size of 1." << std::endl; exit(0);
    }

}

}