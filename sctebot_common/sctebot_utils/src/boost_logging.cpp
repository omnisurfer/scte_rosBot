//
// Created by user on 12/19/21.
//

#include "sctebot_utils/boost_logging.h"

void ScteBotBoostLogger::init_boost_logging() {

    boost::shared_ptr<logging::core> core = logging::core::get();

    boost::shared_ptr<sinks::text_ostream_backend> backend = boost::make_shared<sinks::text_ostream_backend>();

    //backend->add_stream(boost::make_shared<std::ofstream>("sample.log"));
    backend->add_stream(boost::shared_ptr< std::ostream >(&std::clog, boost::null_deleter()));

    typedef sinks::synchronous_sink<sinks::text_ostream_backend> sink_t;
    boost::shared_ptr<sink_t> sink(new sink_t(backend));

    sink->set_filter(
            logging::trivial::severity >= logging::trivial::debug
            );

    sink->set_formatter(
            expr::stream
                    << "[" << expr::format_date_time< boost::posix_time::ptime >("TimeStamp", "%Y-%m-%d %H:%M:%S") << "]"
                    << "[" << boost::log::trivial::severity << "]"
                    << "[" << logging::expressions::attr<logging::attributes::current_thread_id::value_type>("ThreadID") << "]"
                    << "[" << logging::expressions::attr<logging::attributes::current_process_id::value_type>("ProcessID") << "]"
                    << expr::smessage
    );

    core->add_sink(sink);

    boost::log::add_common_attributes();

    core->set_logging_enabled(true);
}

int main(int argc, char* argv[]) {

    ScteBotBoostLogger sctebot_boost_logger = ScteBotBoostLogger();

    sctebot_boost_logger.init_boost_logging();

    std::cout << "HELLO WORLD" << std::endl;

    BOOST_LOG_TRIVIAL(debug) << "TEST";

}