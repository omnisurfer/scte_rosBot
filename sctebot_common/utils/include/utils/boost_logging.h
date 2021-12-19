//
// Created by user on 12/19/21.
//

#ifndef BOOST_LOGGING_H
#define BOOST_LOGGING_H

#include <fstream>
#include <iomanip>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <boost/core/null_deleter.hpp>
#include <boost/smart_ptr/make_shared_object.hpp>
#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

namespace logging = boost::log;
namespace src = boost::log::sources;
namespace expr = boost::log::expressions;
namespace sinks = boost::log::sinks;
namespace keywords = boost::log::keywords;

using namespace logging::trivial;

void init_boost_logging() {

    boost::shared_ptr<logging::core> core = logging::core::get();

    boost::shared_ptr<sinks::text_ostream_backend> backend = boost::make_shared<sinks::text_ostream_backend>();

    //backend->add_stream(boost::make_shared<std::ofstream>("sample.log"));
    backend->add_stream(boost::shared_ptr< std::ostream >(&std::clog, boost::null_deleter()));

    typedef sinks::synchronous_sink<sinks::text_ostream_backend> sink_t;
    boost::shared_ptr<sink_t> sink(new sink_t(backend));
    core->add_sink(sink);
}

#endif //BOOST_LOGGING_H
