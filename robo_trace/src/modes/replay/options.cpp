// Base
#include "robo_trace/modes/replay/options.hpp"
// Std
#include <sstream>
// Boost
#include "boost/program_options.hpp"


namespace po = boost::program_options;

namespace robo_trace {


PlayerOptions::PlayerOptions() {
    //
};

PlayerOptions::~PlayerOptions() = default;


void PlayerOptions::load(const ros::NodeHandle& node_handle) {

    ros::NodeHandle player_namespace(node_handle, "playback");
    ros::NodeHandle publish_clock_namespace(node_handle, "clock");
    
    player_namespace.getParam("topic_prefix", m_topic_prefix);
    
    if (player_namespace.hasParam("clock")) {

        double publish_clock_frequency;   
        player_namespace.getParam("clock", publish_clock_frequency);

        m_publish_clock_frequency = publish_clock_frequency;

    }
    
}

void PlayerOptions::setup(po::options_description& description) {

    description.add_options()
        ("prefix,p", po::value<std::string>()->default_value(""), "prefixes all output topics in replay")
        //("quiet,q", "suppress console output")
        ("immediate,i", "play back all messages without waiting")
        //("pause", "start in paused mode")
        ("queue", po::value<int>()->default_value(100), "use an outgoing queue of size SIZE")
        ("clock", po::value<float>(), "Publish clock time with a frequency of HZ")
        ("delay,d", po::value<float>()->default_value(0.2f), "sleep SEC seconds after every advertise call (to allow subscribers to connect)")
        //("rate,r", po::value<float>()->default_value(1.0f), "multiply the publish rate by FACTOR")
        ("start,s", po::value<float>(), "start SEC seconds into the record")
        ("duration,u", po::value<float>(), "play only SEC seconds from the record")
        //("skip-empty", po::value<float>(), "skip regions in the record with no messages for more than SEC seconds")
        //("loop,l", "loop playback")
        //("keep-alive,k", "keep alive past end of bag (useful for publishing latched topics)")
        //("try-future-version", "still try to open a bag file, even if the version is not known to the player")
        ("topics", po::value< std::vector<std::string> >()->multitoken(), "topics to play back")
        //("pause-topics", po::value< std::vector<std::string> >()->multitoken(), "topics to pause playback on")
        //("bags", po::value< std::vector<std::string> >(), "bag files to play back from")
        ("wait-for-subscribers", "wait for at least one subscriber on each topic before publishing")
        //("rate-control-topic", po::value<std::string>(), "watch the given topic, and if the last publish was more than <rate-control-max-delay> ago, wait until the topic publishes again to continue playback")
        //("rate-control-max-delay", po::value<float>()->default_value(1.0f), "maximum time difference from <rate-control-topic> before pausing");
        ;

}



void PlayerOptions::load(po::variables_map& options) {

    if (options.count("prefix")) {
        m_topic_prefix = options["prefix"].as<std::string>();
    }

    if (options.count("immediate")) {
        m_all_at_once = true;
    }

    if (options.count("queue")) {
        m_topic_queue_size = options["queue"].as<int>();
    }

    if (options.count("start")) {
      m_time_start = options["start"].as<float>();
    }

    if (options.count("duration")) {
      m_time_duration = options["duration"].as<float>();
    }

    if (options.count("delay")) {
        m_wait_duration_after_advertise = ros::WallDuration(options["delay"].as<float>());
    }

    if (options.count("wait-for-subscribers")) {
        m_wait_for_all_topics_subscribed = true;
    }

    if (options.count("topics")) {
      
        std::vector<std::string> topics = options["topics"].as<std::vector<std::string>>();
     
        for (std::vector<std::string>::iterator topic_iterator = topics.begin(); topic_iterator != topics.end(); ++topic_iterator) {
            m_replay_topics.push_back(*topic_iterator);
        }

    }

    if (options.count("clock")) {
        m_publish_clock_frequency = options["hz"].as<double>();
    }

}   

void PlayerOptions::validate() {

    if (m_topic_queue_size < 1) {
        std::cout << "Topic queue size must be greater than 0." << std::endl; exit(0);
    }


}

}