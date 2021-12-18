//
// Created by user on 12/17/21.
//

#ifndef PCA9685_H
#define PCA9685_H

#include <iostream>
#include <string>
#include <thread>
#include <condition_variable>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

//https://torjo.com/log2/doc/html/index.html
namespace logging = boost::log;

#include "i2c_linux/i2c_linux.h"

class Pca9685LEDController {

private:

    class Addresses {

    };

    class Commands {

    };

    int _i2c_bus_number{};
    int _i2c_device_address{};
    int _sensor_update_period_ms{};
    std::string _device_name;

    context_t _i2c_device_context{};

    typedef void (*host_callback_function)(
            int x, int y
    );

    bool run_servo_status_thread = false;
    std::condition_variable run_servo_status_thread_cv;
    std::mutex run_servo_status_thread_mutex;
    std::thread servo_status_thread;

    host_callback_function _host_callback_function{};

    int _connect_to_device() {

        /*
         * setup the i2c context and connect
         */
        _i2c_device_context = {0};
        if(!i2c_dev_open(&_i2c_device_context, _i2c_bus_number, _i2c_device_address)) {
            //BOOST_LOG_TRIVIAL(error) << this->_device_name << ": failed to open device";
            return 0;
        }

        if(!i2c_is_connected(&_i2c_device_context)) {
            //BOOST_LOG_TRIVIAL(error) << this->_device_name << ": failed to connect to device";
            return 0;
        }

        /*
        // try read chip id
        uint8_t chip_id[1] = {0};
        buffer_t inbound_message = {
                .bytes = chip_id,
                .size = sizeof(chip_id)
        };

        uint8_t register_address;
        register_address = Bmp180Pressure::Addresses::DataRegisters::CHIP_ID;
        i2c_recv(&_i2c_device_context, &inbound_message, register_address);

        if(chip_id[0] != Bmp180Pressure::MagicNumbers::ChipId::CHIP_ID) {
            BOOST_LOG_TRIVIAL(error) << this->_device_name << ": failed to read device chip id";
            return 0;
        }
        */

        return 1;
    }

    void _servo_status_worker();

    int _init_device() {

        std::lock_guard<std::mutex> run_lock(this->run_servo_status_thread_mutex);
        this->run_servo_status_thread = true;
        this->run_servo_status_thread_cv.notify_all();

        return 1;
    }

public:

    Pca9685LEDController() = default;

    ~Pca9685LEDController() {

        std::unique_lock<std::mutex> run_lock(this->run_servo_status_thread_mutex);
        this->run_servo_status_thread = false;
        this->run_servo_status_thread_cv.notify_all();
        run_lock.unlock();

        if(servo_status_thread.joinable()) {
            servo_status_thread.join();
        }

    }

    int config_device(
            int bus_number,
            int device_address,
            int update_period_ms,
            std::string device_name,
            host_callback_function
            function_pointer
    ) {
        _i2c_bus_number = bus_number;
        _i2c_device_address = device_address;
        _sensor_update_period_ms = update_period_ms;
        _device_name = std::move(device_name);

        _host_callback_function = function_pointer;

        servo_status_thread = std::thread(&Pca9685LEDController::_servo_status_worker, this);

        return 0;
    }

    int connect_to_device() {
        int status = 1;

        status &= this->_connect_to_device();

        return status;
    }

    int init_device() {

        return  this->_init_device();

    }

};

#endif //PCA9685_H
