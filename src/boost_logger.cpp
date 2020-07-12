#include "boost_logger.hpp"





namespace expr = boost::log::expressions;


typedef boost::log::sinks::asynchronous_sink< boost::log::sinks::text_ostream_backend > text_sink;

bool B_Log::statically_init = false;
boost::shared_ptr< text_sink > B_Log::sink = boost::make_shared< text_sink >();

void B_Log::static_init() {

    
    // set where to log to
    sink->locked_backend()->add_stream(boost::shared_ptr<std::ostream>(&std::clog, boost::null_deleter()));
    // set default format
    set_default_format();

    // default filter level
    sink->set_filter(severity >= Info);

    boost::log::add_common_attributes();

    boost::log::core::get()->add_global_attribute("Timeline", boost::log::attributes::timer());
    boost::log::core::get()->add_global_attribute("ThreadID", boost::log::attributes::current_thread_id());
    boost::log::core::get()->add_sink(sink);

    statically_init = true;
}






// default constructor: logging to std::clog with default format
B_Log::B_Log() {

    if(B_Log::statically_init != true) { 
        BOOST_LOG_TRIVIAL(error) << "\033[0;31mlogger static initializer has not yet been called\033[0m";
    }
    // construct the logger
    slog = slog_ptr(new boost::log::sources::severity_logger<severity_level>);

    
}
    

void B_Log::set_shorter_format() {
    sink->set_formatter
    (
        expr::stream
            << "<" << severity << "> "
            << expr::if_(expr::has_attr(tag_attr))
               [
                    expr::stream << "[" << tag_attr << "] "
               ]
            << expr::smessage
    );

}

void B_Log::set_shortest_format() {
    sink->set_formatter
    (
        expr::stream
            << expr::smessage
    );
}

void B_Log::set_even_shorter_format() {
    sink->set_formatter
    (
        expr::stream
            << expr::if_(expr::has_attr(tag_attr))
               [
                    expr::stream << "[" << tag_attr << "] "
               ]
            << expr::smessage
    );
}

void B_Log::set_default_format() {
        // set default format
    sink->set_formatter
    (
        expr::stream
            << std::dec << std::setw(6) << std::setfill('0') 
                << expr::attr< unsigned int >("LineID") 
                << std::dec << std::setfill(' ') << " "
            << "[ThreadID: " 
                << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID")<< "]"
                
            << expr::if_(expr::has_attr(timeline))
               [
                    expr::stream << "[Time: " << timeline << "] "
               ]
            << ": <" << severity << "> "
            << expr::if_(expr::has_attr(tag_attr))
               [
                    expr::stream << "[" << tag_attr << "] "
               ]
            << expr::smessage
            
    );
}

void B_Log::add_tag(std::string tag) {
    slog->add_attribute(LOG_TAG(tag));
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