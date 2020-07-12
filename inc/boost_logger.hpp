#ifndef __BOOST_LOGGER_H
#define __BOOST_LOGGER_H

#include "boost/log/utility/formatting_ostream.hpp"
#include <boost/log/sinks/async_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>
#include <boost/core/null_deleter.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <ostream>

enum severity_level
{
    Trace,
    Debug,
    Info,
    Warning,
    Error,
    Fatal
};
BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", severity_level);

std::ostream& operator<< (std::ostream& strm, severity_level level);

class B_Log {
private:
    typedef boost::shared_ptr<boost::log::sources::severity_logger<severity_level>> slog_ptr;
    typedef boost::log::sinks::asynchronous_sink< boost::log::sinks::text_ostream_backend > text_sink;
    

    
public:
    severity_level sev = Info;
    slog_ptr slog;
    boost::shared_ptr< text_sink > sink = boost::make_shared< text_sink >();

    // default constructor: logging to std::clog
    B_Log();

    void add_tag(std::string& tag);
    void set_neat_format();
    B_Log& operator()(severity_level sev);

    void log(severity_level sev, std::string str);

};

B_Log& operator<<(B_Log& logger, std::string& str);
B_Log& operator<<(B_Log& logger, const char* str);





#endif