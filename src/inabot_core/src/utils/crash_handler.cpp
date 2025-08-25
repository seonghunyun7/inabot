
#include "crash_handler.hpp"
#include <rclcpp/rclcpp.hpp>

extern rclcpp::Executor* global_executor;

void CrashHandler::setup() {
    std::signal(SIGSEGV, handleSignal);
    std::signal(SIGABRT, handleSignal);
    std::signal(SIGFPE,  handleSignal);
    std::signal(SIGILL,  handleSignal);
    std::signal(SIGBUS,  handleSignal);

    std::signal(SIGINT,  handleSignal);
    std::signal(SIGTSTP, handleSignal);
    std::signal(SIGTERM, handleSignal);
}

void CrashHandler::handleSignal(int sig) {
    switch (sig) {
        case SIGINT:
        case SIGTERM:
        case SIGTSTP:
            std::cout << "Received termination signal (" << sig << "), shutting down cleanly..." << std::endl;
            if (global_executor) global_executor->cancel();
            rclcpp::shutdown();
            break;

        case SIGSEGV: std::cerr << "Segmentation fault (invalid memory access)"; logStackTrace(sig); std::exit(sig); break;
        case SIGABRT: std::cerr << "Abort signal (assert failure?)"; logStackTrace(sig); std::exit(sig); break;
        case SIGFPE:  std::cerr << "Floating point exception"; logStackTrace(sig); std::exit(sig); break;
        case SIGILL:  std::cerr << "Illegal instruction"; logStackTrace(sig); std::exit(sig); break;
        case SIGBUS:  std::cerr << "Bus error"; logStackTrace(sig); std::exit(sig); break;
        default:      std::cerr << "Unknown signal"; logStackTrace(sig); std::exit(sig); break;
    }
}

void CrashHandler::logStackTrace(int sig) {
    const char* crash_dir_env = std::getenv("CRASH_DIR");
    std::string crash_dir = crash_dir_env ? crash_dir_env : "/tmp";

    std::time_t t = std::time(nullptr);
    char timestamp[64];
    std::strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", std::localtime(&t));

    std::ostringstream file_name;
    file_name << crash_dir << "/lrbot2_crash_" << timestamp << ".log";
    std::ofstream log_file(file_name.str());

    std::ostringstream oss;
    oss << "====  signalHandler() : " << sig << "  ====" << std::endl;
    oss << "Time: " << timestamp << std::endl;
    oss << boost::stacktrace::stacktrace();
    oss << "===================================" << std::endl;

    std::cout << "\033[1m\033[33m" << oss.str() << "\033[0m";
    if (log_file.is_open()) {
        log_file << oss.str();
    } else {
        std::cerr << "Failed to write stacktrace to log file: " << file_name.str() << std::endl;
    }
}
