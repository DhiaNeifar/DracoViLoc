#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <fstream>
#include <sstream>
#include <string>
#include <sys/resource.h>
#include <unistd.h>

// From odas demo client
extern "C"
{
#include <odas.h>
#include <parameters.h>
#include <configs.h>
#include <objects.h>
#include <threads.h>
#include <profiler.h>
}

namespace
{

std::string core_limit_string(rlim_t value)
{
    return value == RLIM_INFINITY ? "unlimited" : std::to_string(value);
}

std::string read_core_pattern()
{
    std::ifstream stream("/proc/sys/kernel/core_pattern");
    std::string pattern;
    std::getline(stream, pattern);
    return pattern.empty() ? "unavailable" : pattern;
}

void configure_core_dumps(const rclcpp::Logger & logger)
{
    struct rlimit limit {};
    if (getrlimit(RLIMIT_CORE, &limit) != 0) {
        RCLCPP_WARN(logger, "Could not read RLIMIT_CORE");
        return;
    }

    const rlim_t original_soft_limit = limit.rlim_cur;
    if (limit.rlim_cur != limit.rlim_max) {
        limit.rlim_cur = limit.rlim_max;
        if (setrlimit(RLIMIT_CORE, &limit) != 0) {
            RCLCPP_WARN(
                logger, "Could not raise core-dump soft limit (current=%s hard=%s)",
                core_limit_string(original_soft_limit).c_str(),
                core_limit_string(limit.rlim_max).c_str());
        }
    }

    if (getrlimit(RLIMIT_CORE, &limit) == 0) {
        RCLCPP_INFO(
            logger, "Core dumps: soft=%s hard=%s pattern=%s",
            core_limit_string(limit.rlim_cur).c_str(),
            core_limit_string(limit.rlim_max).c_str(), read_core_pattern().c_str());
    }
}

}  // namespace

int main(int argc, char** argv)
{
    // libODAS uses stdio directly. Line buffering ensures its last complete
    // diagnostic line reaches the parent bridge before an abnormal exit.
    setvbuf(stdout, nullptr, _IOLBF, 0);
    setvbuf(stderr, nullptr, _IONBF, 0);
    std::set_terminate([]() {
        std::fprintf(stderr, "FATAL: odas_core_node terminated by an unhandled C++ exception\n");
        std::abort();
    });

    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("odas_core_node");

    RCLCPP_INFO(node->get_logger(), "ODAS core pid=%d", static_cast<int>(getpid()));
    configure_core_dumps(node->get_logger());

    std::string configFile = node->declare_parameter("configuration_path", "");

    RCLCPP_INFO(node->get_logger(), "Using configuration file = %s", configFile.c_str());

    // +------------------------------------------------------+
    // | Multiple threads                                     |
    // +------------------------------------------------------+


    // +----------------------------------------------------------+
    // | Variables                                                |
    // +----------------------------------------------------------+


    // +------------------------------------------------------+
    // | Objects                                              |
    // +------------------------------------------------------+
    aobjects* aobjs = NULL;

    // +------------------------------------------------------+
    // | Configurations                                       |
    // +------------------------------------------------------+
    configs* cfgs = NULL;


    // +--------------------------------------------------+
    // | Configure                                        |
    // +--------------------------------------------------+
    RCLCPP_INFO(node->get_logger(), "| + Initializing configurations...... ");
    cfgs = configs_construct(configFile.c_str());


    // +--------------------------------------------------+
    // | Construct                                        |
    // +--------------------------------------------------+
    RCLCPP_INFO(node->get_logger(), "| + Initializing objects............. ");
    aobjs = aobjects_construct(cfgs);


    // +--------------------------------------------------+
    // | Launch threads                                   |
    // +--------------------------------------------------+
    RCLCPP_INFO(node->get_logger(), "| + Launch threads................... ");
    threads_multiple_start(aobjs);


    RCLCPP_INFO(node->get_logger(), "| + ROS SPINNING................... ");

    const auto start_time = std::chrono::steady_clock::now();
    auto heartbeat_timer = node->create_wall_timer(std::chrono::seconds(10), [node, start_time]() {
        struct rusage usage {};
        const long max_rss_kb = getrusage(RUSAGE_SELF, &usage) == 0 ? usage.ru_maxrss : -1;
        const double uptime = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - start_time).count();
        RCLCPP_INFO(
            node->get_logger(), "ODAS core heartbeat uptime=%.1fs max_rss_kb=%ld",
            uptime, max_rss_kb);
    });
    (void)heartbeat_timer;

    // Start ros loop
    rclcpp::spin(node);

    // +--------------------------------------------------+
    // | Wait                                             |
    // +--------------------------------------------------+
    RCLCPP_INFO(node->get_logger(), "| + Stop threads.................. ");
    threads_multiple_stop(aobjs);
    RCLCPP_INFO(node->get_logger(), "| + Threads join.................. ");
    threads_multiple_join(aobjs);


    // +--------------------------------------------------+
    // | Free memory                                      |
    // +--------------------------------------------------+
    RCLCPP_INFO(node->get_logger(), "| + Free memory...................... ");


    aobjects_destroy(aobjs);
    configs_destroy(cfgs);
    RCLCPP_INFO(node->get_logger(), "Done!");

    rclcpp::shutdown();

    return 0;
}
