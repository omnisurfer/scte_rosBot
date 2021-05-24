//
// Created by user on 5/23/21.
//

#include "l3gd20.h"

int L3gd20::_init_l3gd20() {

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::debug
    );

    std::lock_guard<std::mutex> lk(this->l3gd20_data_capture_thread_run_mutex);
    this->l3gd20_data_capture_thread_run_cv.notify_one();
    this->run_l3gd20_data_capture_thread = true;

    return 0;
}

void L3gd20::_l3gd20_data_capture_worker() {
    BOOST_LOG_TRIVIAL(debug) << "_l3gd20_data_capture_worker starting";

    std::unique_lock<std::mutex> data_lock(this->l3gd20_data_capture_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "waiting to run...";
    this->l3gd20_data_capture_thread_run_cv.wait(data_lock);
    data_lock.unlock();

    data_lock.lock();
    while(this->run_l3gd20_data_capture_thread) {
        data_lock.unlock();

        // DO STUFF

        data_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "_l3gd20_data_capture_worker exiting";
}