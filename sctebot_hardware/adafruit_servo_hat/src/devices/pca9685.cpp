//
// Created by user on 12/17/21.
//

#include "pca9685.h"

int Pca9685LEDController::_init_device() {

    buffer_t inbound_message;

    int init_ok = 0;
    uint8_t register_address = 0x00; //(Bmp180Pressure::Addresses::CalibrationCoefficients::AC1 >> 8) & 0xff;

    uint8_t temp_data_buffer[3];

    inbound_message = {
            .bytes = temp_data_buffer,
            .size = sizeof(temp_data_buffer)
    };

    if(receive_i2c(&inbound_message, register_address)) {

    }
    else {
        init_ok = 1;
    }

    std::lock_guard<std::mutex> run_lock(this->run_servo_status_thread_mutex);
    this->run_servo_status_thread = true;
    this->run_servo_status_thread_cv.notify_all();

    return init_ok;
}

void Pca9685LEDController::_servo_management_worker() {

    std::unique_lock<std::mutex> run_lock(this->run_servo_status_thread_mutex);
    this->run_servo_status_thread_cv.wait(run_lock);
    run_lock.unlock();

    run_lock.lock();
    while(this->run_servo_status_thread) {
        run_lock.unlock();

        this->_host_callback_function(1, 2);

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_sensor_update_period_ms));

        run_lock.lock();
    }
}

int Pca9685LEDController::receive_i2c(buffer_t *data, uint8_t register_address) {

    std::lock_guard<std::mutex> i2c_device_lock(_i2c_device_mutex);
    int status = i2c_recv(&_i2c_device_context, data, register_address);

    return status;
}

int Pca9685LEDController::send_i2c(buffer_t *data, uint8_t register_address) {

    std::lock_guard<std::mutex> i2c_device_lock(_i2c_device_mutex);
    int status = i2c_send(&_i2c_device_context, data, register_address);

    return status;
}

int Pca9685LEDController::open_i2c_dev(int device_number, int slave_address) {

    std::lock_guard<std::mutex> i2c_device_lock(_i2c_device_mutex);
    int status = i2c_dev_open(&_i2c_device_context, device_number, slave_address);

    return status;
}

int Pca9685LEDController::is_i2c_dev_connected() {

    std::lock_guard<std::mutex> i2c_device_lock(_i2c_device_mutex);
    int status = i2c_is_connected(&_i2c_device_context);

    return status;
}

void Pca9685LEDController::close_i2c_dev(int bus_number) {

    std::lock_guard<std::mutex> i2c_device_lock(_i2c_device_mutex);
    i2c_dev_close(&_i2c_device_context, bus_number);
}

void handle_servo_callback(int x, int y) {

    //std::cout << "servo callback called!" << std::endl;

    // Boost logging causing thread race conditions (all of them???)
    //BOOST_LOG_TRIVIAL(debug) << "handle_servo_callback " << x << " " << y;
    //BOOST_LOG_TRIVIAL(info) << "callback info message";

}

int main(int argc, char* argv[]) {

    ScteBotBoostLogger sctebot_boost_logger = ScteBotBoostLogger();
    sctebot_boost_logger.init_boost_logging();

    BOOST_LOG_TRIVIAL(debug) << "pca9685 debug message";

    std::unique_ptr<Pca9685LEDController> pca9685DeviceHandle;

    pca9685DeviceHandle.reset(new Pca9685LEDController());

    int i2c_bus_number = 0;
    int i2c_device_address = 0x40;
    int update_period_ms = 1000;

    bool init_ok = true;

    pca9685DeviceHandle->config_device(
            i2c_bus_number,
            i2c_device_address,
            update_period_ms,
            "pca9685_servo",
            handle_servo_callback
            );

    if(pca9685DeviceHandle->connect_to_device()) {
        std::cout << "pca9685 failed to connect" << std::endl;
        return 0;
    }
    else {

        pca9685DeviceHandle->init_device();

        std::cout << "press any key to exit" << std::endl;
        std::cin.get();

    }

}