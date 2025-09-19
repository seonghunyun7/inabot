#include "rclcpp/rclcpp.hpp"
#include "acs_node.hpp"
#include <signal.h>
#include <iostream>

#include <ctime>
#include <boost/stacktrace.hpp>


std::shared_ptr<AcsNode> g_node = nullptr;
 
namespace SignalHandler {

void logStackTrace(int sig)
{
    std::time_t t = std::time(nullptr);
    char timestamp[64];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&t));

    std::ostringstream oss;
    oss << "==== signalHandler() : " << sig << " ====" << std::endl;
    oss << "Time: " << timestamp << std::endl;
    oss << boost::stacktrace::stacktrace();
    oss << "===================================" << std::endl;

    std::cout << "\033[1m\033[33m" << oss.str() << "\033[0m";
}

void signalhandler(int sig)
{
    // MQTT 종료
    if (g_node) {
        g_node->shutdownMqtt();
    }

    switch (sig) {
        case SIGINT:
        //case SIGTERM:
        case SIGTSTP:
            std::cout << "Received termination signal (" << sig << "), shutting down cleanly..." << std::endl;
            rclcpp::shutdown();
            break;

        //SIGTERM을 명시적으로 구분 → power off나 시스템 종료 감지.
        case SIGTERM:
            std::cout << "Received SIGTERM (system shutdown / power off), performing cleanup..." << std::endl;
            rclcpp::shutdown();
            break;

        case SIGSEGV: std::cerr << "Segmentation fault (invalid memory access)\n"; logStackTrace(sig); std::exit(sig); break;
        case SIGABRT: std::cerr << "Abort signal (assert failure?)\n"; logStackTrace(sig); std::exit(sig); break;
        case SIGFPE:  std::cerr << "Floating point exception\n"; logStackTrace(sig); std::exit(sig); break;
        case SIGILL:  std::cerr << "Illegal instruction\n"; logStackTrace(sig); std::exit(sig); break;
        case SIGBUS:  std::cerr << "Bus error\n"; logStackTrace(sig); std::exit(sig); break;
        default:      std::cerr << "Unknown signal\n"; logStackTrace(sig); std::exit(sig); break;
    }
}

void setup() {
    std::signal(SIGSEGV, signalhandler);
    std::signal(SIGABRT, signalhandler);
    std::signal(SIGFPE,  signalhandler);
    std::signal(SIGILL,  signalhandler);
    std::signal(SIGBUS,  signalhandler);

    std::signal(SIGINT,  signalhandler);
    std::signal(SIGTSTP, signalhandler);
    std::signal(SIGTERM, signalhandler);
}

} // namespace SignalHandler

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    SignalHandler::setup();

    g_node = std::make_shared<AcsNode>();
    g_node->init();

    //멀티 스레드 Executor
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(g_node);
    executor.spin();

    g_node->shutdownMqtt();
    rclcpp::shutdown();
    return 0;
}
