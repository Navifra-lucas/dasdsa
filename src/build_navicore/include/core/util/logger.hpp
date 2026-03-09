#ifndef NC_LOGGER_H
#define NC_LOGGER_H

#include <boost/asio.hpp>
#include <boost/core/null_deleter.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/format.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/attributes/scoped_attribute.hpp>
#include <boost/log/common.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/utility/manipulators/add_value.hpp>
#include <boost/noncopyable.hpp>
#include <boost/phoenix.hpp>
#include <boost/serialization/singleton.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include <iostream>
#include <ostream>
#include <sstream>
#include <string>

// Here we define our application severity levels.
enum severity_level
{
    trace,
    debug,
    info,
    warning,
    error,
    fatal
};

BOOST_LOG_INLINE_GLOBAL_LOGGER_DEFAULT(slg, boost::log::sources::severity_logger<severity_level>)

typedef boost::log::sinks::asynchronous_sink<boost::log::sinks::text_ostream_backend> text_sink;
typedef boost::log::sinks::asynchronous_sink<boost::log::sinks::text_file_backend> file_sink;

#define LOG_TRACE(str, ...)                                                                             \
    BOOST_LOG_SEV(slg::get(), severity_level::trace)                                                    \
        << NaviFra::Logger::get().format(str, ##__VA_ARGS__) << boost::log::add_value("File", __FILE__) \
        << boost::log::add_value("Line", __LINE__) << boost::log::add_value("Function", __PRETTY_FUNCTION__);

#define LOG_INFO(str, ...)                                                                              \
    BOOST_LOG_SEV(slg::get(), severity_level::info)                                                     \
        << NaviFra::Logger::get().format(str, ##__VA_ARGS__) << boost::log::add_value("File", __FILE__) \
        << boost::log::add_value("Line", __LINE__) << boost::log::add_value("Function", __PRETTY_FUNCTION__);

#define LOG_DEBUG(str, ...)                                                                             \
    BOOST_LOG_SEV(slg::get(), severity_level::debug)                                                    \
        << NaviFra::Logger::get().format(str, ##__VA_ARGS__) << boost::log::add_value("File", __FILE__) \
        << boost::log::add_value("Line", __LINE__) << boost::log::add_value("Function", __PRETTY_FUNCTION__);

#define LOG_WARNING(str, ...)                                                                           \
    BOOST_LOG_SEV(slg::get(), severity_level::warning)                                                  \
        << NaviFra::Logger::get().format(str, ##__VA_ARGS__) << boost::log::add_value("File", __FILE__) \
        << boost::log::add_value("Line", __LINE__) << boost::log::add_value("Function", __PRETTY_FUNCTION__);

#define LOG_ERROR(str, ...)                                                                             \
    BOOST_LOG_SEV(slg::get(), severity_level::error)                                                    \
        << NaviFra::Logger::get().format(str, ##__VA_ARGS__) << boost::log::add_value("File", __FILE__) \
        << boost::log::add_value("Line", __LINE__) << boost::log::add_value("Function", __PRETTY_FUNCTION__);

#define LOG_FATAL(str, ...)                                                                             \
    BOOST_LOG_SEV(slg::get(), severity_level::fatal)                                                    \
        << NaviFra::Logger::get().format(str, ##__VA_ARGS__) << boost::log::add_value("File", __FILE__) \
        << boost::log::add_value("Line", __LINE__) << boost::log::add_value("Function", __PRETTY_FUNCTION__);

#define NLOG(lvl)                                                                                                        \
    BOOST_LOG_SEV(slg::get(), lvl) << boost::log::add_value("File", __FILE__) << boost::log::add_value("Line", __LINE__) \
                                   << boost::log::add_value("Function", __FUNCTION__)

namespace NaviFra {
class Logger
    : public boost::noncopyable
    , public boost::serialization::singleton<Logger> {
public:
    Logger();
    virtual ~Logger();
    friend class boost::serialization::singleton<Logger>;

    static Logger& get() { return boost::serialization::singleton<Logger>::get_mutable_instance(); }

public:
    void SetSeverityMin(severity_level lv);
    std::string format(const char* str, ...);

private:
    boost::shared_ptr<text_sink> consoleSink_;
    boost::shared_ptr<file_sink> fileSink_;
};
}  // namespace NaviFra
#endif