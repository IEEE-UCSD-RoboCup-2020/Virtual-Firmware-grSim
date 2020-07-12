#include "boost_logger.hpp"


#include <iostream>



BOOST_LOG_ATTRIBUTE_KEYWORD(line_id, "LineID", unsigned int)

BOOST_LOG_ATTRIBUTE_KEYWORD(tag_attr, "Tag", std::string);
#define LOG_TAG(str) "Tag", boost::log::attributes::constant< std::string >(str)

BOOST_LOG_ATTRIBUTE_KEYWORD(timeline, "Timeline", boost::log::attributes::timer::value_type)


// default constructor: logging to std::clog with default format
B_Log::B_Log() {
        // set where to log to
    sink->locked_backend()->add_stream(boost::shared_ptr<std::ostream>(&std::clog, boost::null_deleter()));
    // set default format
    sink->set_formatter
    (
        boost::log::expressions::stream
            << std::hex << std::setw(8) << std::setfill('0') << line_id << std::dec << std::setfill(' ')
            << boost::log::expressions::if_(boost::log::expressions::has_attr(timeline))
               [
                    boost::log::expressions::stream << "[" << timeline << "] "
               ]
            << ": <" << severity << ">\t"
            << boost::log::expressions::if_(boost::log::expressions::has_attr(tag_attr))
               [
                    boost::log::expressions::stream << "[" << tag_attr << "] "
               ]
            << boost::log::expressions::smessage
    );

    BOOST_LOG_SCOPED_THREAD_ATTR("Timeline", boost::log::attributes::timer());
    boost::log::add_common_attributes();

    sink->set_filter(severity >= Info);

    boost::log::core::get()->add_sink(sink);


    // construct the logger
    slog = slog_ptr(new boost::log::sources::severity_logger<severity_level>);
}
    


void B_Log::add_tag(std::string& tag) {
    slog->add_attribute(LOG_TAG(tag));
}

void B_Log::set_neat_format() {
    sink->set_formatter
    (
        boost::log::expressions::stream
            << ": <" << severity << ">\t"
            << boost::log::expressions::if_(boost::log::expressions::has_attr(tag_attr))
               [
                    boost::log::expressions::stream << "[" << tag_attr << "] "
               ]
            << boost::log::expressions::smessage
    );

}

B_Log& B_Log::operator()(severity_level sev) {
    this->sev = sev;
    return *this;
}


B_Log& operator<<(B_Log& logger, std::string str)
{
    BOOST_LOG_SEV(*(logger.slog), logger.sev) << str; 
    return logger;
}

B_Log& operator<<(B_Log& logger, const char* str) {
    BOOST_LOG_SEV(*(logger.slog), logger.sev) << std::string(str);
    return logger;
}


std::ostream& operator<< (std::ostream& strm, severity_level level)
{
    static const char* strings[] =
    {
        "trace",
        "debug",
        "info",
        "warning",
        "error",
        "fatal"
    };

    if (static_cast< std::size_t >(level) < sizeof(strings) / sizeof(*strings))
        strm << strings[level];
    else
        strm << static_cast< int >(level);

    return strm;
}




void B_Log::log(severity_level sev, std::string str) {
    BOOST_LOG_SEV(*(this->slog), sev) << std::string(str);
}