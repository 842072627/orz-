#include "Logger.h"
#include <boost/log/utility/setup/console.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/support/date_time.hpp>

namespace logging = boost::log;
namespace keywords = boost::log::keywords;
namespace src = boost::log::sources;
namespace sinks = boost::log::sinks;
namespace expr = boost::log::expressions;
using namespace logging::trivial;
src::severity_logger< severity_level > lg;


Logger::Logger(std::string filename, int severity, bool skip_warning){
    this->core = logging::core::get();
    logging::add_common_attributes();
    if (skip_warning) {
        // Show info and fatal only, skip warning
        logging::core::get()->set_filter(
            logging::trivial::severity >= severity &&
            logging::trivial::severity != logging::trivial::warning
        );
    } else {
        logging::core::get()->set_filter(
            logging::trivial::severity >= severity
        );
    }

    // Set console output format (without thread ID)
    logging::add_console_log(
        std::clog,
        keywords::format = (
            expr::stream
                << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "]"
                << " [" << logging::trivial::severity << "]"
                << "    " << expr::smessage
        )
    );

    if (filename != "")
        logging::add_file_log
        (
            keywords::file_name = filename,
            keywords::format = "[%TimeStamp%]: *%Severity%* %Message%"
        );
}

void Logger::flush(){
    this->core->flush();
}


void Logger::log_info(std::string input)
{

    BOOST_LOG_SEV(lg, info) << input;
}


void Logger::log_info(std::string input, int timestep)
{
    log_info("[timetick=" + std::to_string(timestep) + "] " + input);
}


void Logger::log_fatal(std::string input, int timestep)
{
    log_fatal("[timetick=" + std::to_string(timestep) + "] " + input);
}


void Logger::log_fatal(std::string input)
{

    BOOST_LOG_SEV(lg, fatal) << input;
}


void Logger::log_warning(std::string input)
{

    BOOST_LOG_SEV(lg, warning) << input;
}


void Logger::log_warning(std::string input, int timestep)
{
    log_warning("[timetick=" + std::to_string(timestep) + "] " + input);
}
