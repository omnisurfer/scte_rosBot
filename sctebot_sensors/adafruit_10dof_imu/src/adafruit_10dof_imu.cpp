//
// Created by user on 4/9/21.
//

#include "adafruit_10dof_imu.h"

AdaFruit10DoFImu::AdaFruit10DoFImu() {

}

void AdaFruit10DoFImu::_data_capture_execute_management_worker() {

    std::string device_name = this->_device_name;

    BOOST_LOG_TRIVIAL(debug) << device_name << ": _data_capture_execute_management_worker starting";

    BOOST_LOG_TRIVIAL(debug) << device_name << ": _data_capture_execute_management_worker waiting";
    std::unique_lock<std::mutex> execute_management_lock(this->data_capture_execute_management_thread_run_mutex);
    this->data_capture_execute_management_thread_run_cv.wait(execute_management_lock);
    is_running_data_capture_execute_management_thread = true;
    execute_management_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << device_name << ": _data_capture_worker running";
    execute_management_lock.lock();
    while(this->run_data_capture_execute_management_thread) {
        execute_management_lock.unlock();


        std::unique_lock<std::mutex> bmp180_execute_lock(this->bmp180_data_capture_worker_execute_cycle_mutex);
        this->bmp180_data_capture_worker_execute_cycle_cv.notify_all();
        bmp180_execute_lock.unlock();

        std::unique_lock<std::mutex> l3gd20_execute_lock(this->l3gd20_data_capture_worker_execute_cycle_mutex);
        this->l3gd20_data_capture_worker_execute_cycle_cv.notify_all();
        l3gd20_execute_lock.unlock();

        std::unique_lock<std::mutex> lsm303dlhc_accel_execute_lock(this->lsm303dlhc_accel_data_capture_worker_execute_cycle_mutex);
        this->lsm303dlhc_accel_data_capture_worker_execute_cycle_conditional_variable.notify_all();
        lsm303dlhc_accel_execute_lock.unlock();

        std::unique_lock<std::mutex> lsm303dlhc_mag_execute_lock(this->lsm303dlhc_mag_data_capture_worker_execute_cycle_mutex);
        this->lsm303dlhc_mag_data_capture_worker_execute_cycle_conditional_variable.notify_all();
        lsm303dlhc_mag_execute_lock.unlock();

        // small sleep to keep from busy looping
        std::this_thread::sleep_for(std::chrono::microseconds (1000 * 100 * 10));

        execute_management_lock.lock();
    }

}