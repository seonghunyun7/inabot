#include "utils/logger.h"

#include <cassert>
#include <cstdarg>
#include <fstream>
#include <iostream>
#include <string>

using namespace Inabot;
namespace logging = boost::log;
namespace expr = boost::log::expressions;
namespace sinks = boost::log::sinks;
namespace attrs = boost::log::attributes;
namespace src = boost::log::sources;
namespace keywords = boost::log::keywords;


using boost::shared_ptr;

std::string color(severity_level lvl)
{
    std::string color = "\033[0m";
    switch (lvl) {
        case severity_level::info:
            color = "\033[36m";
            break;
        case severity_level::warning:
            color = "\033[33m";
            break;
        case severity_level::fatal:
        case severity_level::error:
            color = "\033[31m";
            break;
        case severity_level::debug:
            color = "\033[90m";
        default:
            break;
    }
    return color;
}

// The formatting logic for the severity level
template <typename CharT, typename TraitsT>
inline std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& strm, severity_level lvl)
{
    static const char* const str[] = {"TRACE", "DEBUG", "INFO", "WARNING", "ERROR", "FATAL"};
    if (static_cast<std::size_t>(lvl) < (sizeof(str) / sizeof(*str)))
        strm << color(lvl) << str[lvl] << "\033[00m";
    else
        strm << static_cast<int>(lvl);
    return strm;
}

std::string file_basename(logging::value_ref<std::string> const& filename)
{
    // Check to see if the attribute value has been found
    if (filename)
        return boost::filesystem::path(filename.get()).filename().string();
    else
        return std::string();
}

namespace Inabot {

Logger::Logger()
{
    consoleSink_.reset(new text_sink);
    {
        text_sink::locked_backend_ptr pBackend = consoleSink_->locked_backend();

        // Next we add streams to which logging records should be output
        shared_ptr<std::ostream> pStream(&std::cout, boost::null_deleter());
        pBackend->add_stream(pStream);
    }

    logging::core::get()->add_sink(consoleSink_);

    consoleSink_->set_formatter(
        expr::stream << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%d.%m.%Y %H:%M:%S.%f") << "] ["
                     << expr::attr<severity_level>("Severity") << "] [" << expr::attr<boost::posix_time::time_duration>("Uptime") << "] ["
                     << boost::phoenix::bind(&file_basename, expr::attr<std::string>("File")) << " "
                     << expr::attr<int>("Line")
                     //<< expr::attr<std::string>("Function") << " " << expr::attr<int>("File") << " " << expr::attr<int>("Line")
                     << "]  " << expr::smessage);  // here goes the log record text

    attrs::counter<unsigned int> RecordID(1);

    char buffer[1024];
    std::string processName;
    ssize_t len = readlink("/proc/self/exe", buffer, sizeof(buffer) - 1);
    if (len != -1) {
        buffer[len] = '\0';
        processName = basename(buffer);
    }

    std::string logfilepath = "";
    if (std::getenv("HOME") != nullptr) {
        logfilepath = std::getenv("HOME");
        logfilepath += "/.ros/logs/" + processName;
    }
    else {
        logfilepath = "/.ros/logs/" + processName;
    }

    // Create a text file sink
    fileSink_.reset(new file_sink(
        keywords::file_name = logfilepath + "/" + processName + ".log",  // file name pattern
        keywords::target_file_name = processName + "_%Y%m%d_%H%M%S_%5N.log",  // file name pattern
        keywords::rotation_size = 1024 * 1024 * 30,  // Rotate at 100 MB
        keywords::open_mode = std::ios_base::app | std::ios_base::out,
        keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),  // Rotate at midnight,
        keywords::auto_flush = true));

    // Set up where the rotated files will be stored
    fileSink_->locked_backend()->set_file_collector(sinks::file::make_collector(
        keywords::target = logfilepath,  // where to store rotated files
        keywords::max_size = static_cast<boost::uintmax_t>(1024) * 1024 * 1024 * 30,  // maximum total size of the stored files, in bytes
        keywords::min_free_space = 1024 * 1024 * 1024,  // minimum free space on the drive, in bytes
        keywords::max_files = 20  // maximum number of stored files
        ));
    fileSink_->locked_backend()->scan_for_files();
    fileSink_->set_formatter(
        expr::stream << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%d.%m.%Y %H:%M:%S.%f") << "] ["
                     << expr::attr<severity_level>("Severity") << "] [" << expr::attr<boost::posix_time::time_duration>("Uptime") << "] ["
                     << boost::phoenix::bind(&file_basename, expr::attr<std::string>("File")) << " "
                     << expr::attr<int>("Line")
                     //<< expr::attr<std::string>("Function") << " " << expr::attr<int>("File") << " " << expr::attr<int>("Line")
                     << "]  " << expr::smessage);
    logging::core::get()->add_sink(fileSink_);

    shared_ptr<file_sink> errorSink(new file_sink(
        keywords::file_name = logfilepath + "/error/" + "error" + ".log",  // file name pattern
        keywords::target_file_name = processName +
            "_error_"
            "%Y%m%d_%H%M%S_%5N.log",  // file name pattern
        keywords::rotation_size = 1024 * 1024 * 30,  // Rotate at 100MB
        keywords::open_mode = std::ios_base::app | std::ios_base::out,
        keywords::time_based_rotation = sinks::file::rotation_at_time_point(0, 0, 0),  // Rotate at midnight,
        keywords::auto_flush = true));

    // Set up where the rotated files will be stored
    errorSink->locked_backend()->set_file_collector(sinks::file::make_collector(
        keywords::target = logfilepath + "/error/",  // where to store rotated files
        keywords::max_size = static_cast<boost::uintmax_t>(1024) * 1024 * 1024 * 30,  // maximum total size of the stored files, in bytes
        keywords::min_free_space = 1024 * 1024 * 1024,  // minimum free space on the drive, in bytes
        keywords::max_files = 20  // maximum number of stored files
        ));
    errorSink->locked_backend()->scan_for_files();
    errorSink->set_formatter(
        expr::stream << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%d.%m.%Y %H:%M:%S.%f") << "] ["
                     << expr::attr<severity_level>("Severity") << "] [" << expr::attr<boost::posix_time::time_duration>("Uptime") << "] ["
                     << boost::phoenix::bind(&file_basename, expr::attr<std::string>("File")) << " "
                     << expr::attr<int>("Line")
                     //<< expr::attr<std::string>("Function") << " " << expr::attr<int>("File") << " " << expr::attr<int>("Line")
                     << "]  " << expr::smessage);

    errorSink->set_filter(boost::log::expressions::attr<severity_level>("Severity") >= severity_level::error);
    logging::core::get()->add_sink(errorSink);
    // Since we intend to count all logging records ever made by the application,
    // this attribute should clearly be global.
    // And similarly add a time stamp
    attrs::local_clock TimeStamp;
    logging::core::get()->add_global_attribute("TimeStamp", TimeStamp);
    boost::log::core::get()->add_global_attribute("Uptime", boost::log::attributes::timer());
}

Logger::~Logger()
{
    logging::core::get()->flush();
    logging::core::get()->remove_all_sinks();
}

void Logger::SetSeverityMin(severity_level lv)
{
    fileSink_->set_filter(boost::log::expressions::attr<severity_level>("Severity") >= lv);
    consoleSink_->set_filter(boost::log::expressions::attr<severity_level>("Severity") >= lv);
}

std::string Logger::format(const char* str, ...)
{
    try {
        va_list args;
        va_start(args, str);
        int length = std::vsnprintf(nullptr, 0, str, args);
        va_end(args);

        va_start(args, str);
        std::vector<char> message(length + 1);  // +1 for the null-terminator
        std::vsnprintf(message.data(), message.size(), str, args);
        va_end(args);

        return message.data();
    }
    catch (std::exception& ex) {
        return "LoggerFormat Error" + std::string(ex.what());
    }
}

}  // namespace Inabot
