//
// Created by user on 6/21/21.
//

#include "lsm303dlhc.h"

int Lsm303dlhc::_init_device() {

    logging::core::get()->set_filter
    (
            logging::trivial::severity >= logging::trivial::debug
    );

    // config TBD

    std::lock_guard<std::mutex> lk(this->data_capture_thread_run_mutex);
    this->data_capture_thread_run_cv.notify_one();
    this->run_data_capture_thread = true;

    return 0;
}

void Lsm303Dlhc::_data_capture_worker() {

}

void Lsm303Dlhc::_mock_device_emulation() {

}
