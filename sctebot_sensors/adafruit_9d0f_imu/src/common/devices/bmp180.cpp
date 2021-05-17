//
// Created by user on 4/9/21.
//

#include "bmp180.h"

int Bmp180::_init_bmp180() {

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::debug
    );

    buffer_t inbound_message = {
            .bytes = _bmp180_calibration_data_buffer,
            .size = sizeof(_bmp180_calibration_data_buffer)
    };

    uint8_t register_address = (Bmp180::Addresses::CalibrationCoefficients::AC1 >> 8) & 0xff;
    if (i2c_recv(&_bmp180_i2c_context, &inbound_message, register_address)) {

        std::cout << "calibration data: " << std::endl;

        for(int i = 0; i < sizeof(_bmp180_calibration_data_buffer); ++i) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)_bmp180_calibration_data_buffer[i] << " ";
        }

        std::cout << std::endl;
    }

    this->_init_bmp180_calibration_coefficients((char *) this->_get_bmp180_calibration_buffer_address(),
                                                this->_get_bmp180_calibration_buffer_size());

    std::lock_guard<std::mutex> lk(this->bmp180_data_capture_thread_run_mutex);
    this->bmp180_data_capture_thread_run_cv.notify_one();
    this->run_bmp180_data_capture_thread = true;

    return 1;
}

void Bmp180::_bmp180_data_capture_worker() {
    std::cout << "_bmp180_data_capture_worker starting" << std::endl;

    std::unique_lock<std::mutex> data_lock(this->bmp180_data_capture_thread_run_mutex);
    std::cout << "waiting to run..." << std::endl;
    this->bmp180_data_capture_thread_run_cv.wait(data_lock);
    data_lock.unlock();

    data_lock.lock();
    while(this->run_bmp180_data_capture_thread) {
        data_lock.unlock();

        this->_get_bmp180_temperature();

        // wait for conversion to finish (Sco low)
        if(!this->_measurement_completed_ok()) {
            BOOST_LOG_TRIVIAL(debug) << "Measurement did not complete ok";
        }

        this->_get_bmp180_pressure();

        // wait for conversion to finish (Sco low)
        if(!this->_measurement_completed_ok()) {
            BOOST_LOG_TRIVIAL(debug) << "Measurement did not complete ok";
        }

        uint16_t long_uncompensated_temperature = this->_get_bmp180_uncompensated_temperature_count();
        uint16_t long_uncompensated_pressure = this->_get_bmp180_uncompensated_pressure();
        uint8_t short_uncompensated_pressure_xlsb = this->_get_bmp180_uncompensated_pressure_xlsb();

        float calculated_temperature = 0.0f;
        this->_calculate_temperature(long_uncompensated_temperature, calculated_temperature);

        float calculated_pressure = 0.0f;
        this->_calculate_pressure(long_uncompensated_pressure, short_uncompensated_pressure_xlsb, calculated_pressure);

        this->_bmp180_host_callback_function(calculated_temperature, calculated_pressure);

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_bmp180_sensor_update_period_ms));

        data_lock.lock();
    }

    std::cout << "_bmp180_data_capture_worker exiting" << std::endl;
}

void Bmp180::_get_bmp180_temperature() {
// command temperature read
    // notify bmp180 to read temp reg
    // wait 4.5ms
    // read reg 0xF6F7
    uint8_t control_register_and_start_meas[1] = {Bmp180::Commands::MeasurementControlValues ::TEMPERATURE};
    buffer_t outbound_message = {
            .bytes = control_register_and_start_meas,
            .size = 1
    };

    uint8_t register_address;

    register_address = Bmp180::Addresses::DataRegisters::CONTROL_MEASUREMENT;
    if (i2c_send(&_bmp180_i2c_context, &outbound_message, register_address)) {
        std::this_thread::sleep_for(std::chrono::microseconds (4500));
    }
    else {
        std::cout << "failed to command temperature read" << std::endl;
    }

    // read back temperature
    uint8_t uncompensated_temperature[2] = {0};
    buffer_t inbound_message = {
            .bytes = uncompensated_temperature,
            .size = sizeof(uncompensated_temperature)
    };

    register_address = (Bmp180::Addresses::DataRegisters::OUTPUT >> 8) & 0xff;
    i2c_recv(&_bmp180_i2c_context, &inbound_message, register_address);

    _bmp180_long_uncompensated_temperature = (uncompensated_temperature[_device_endian_lsb_index] << 8) | uncompensated_temperature[_device_endian_msb_index];
}

void Bmp180::_get_bmp180_pressure() {

    uint8_t register_address;

    // command pressure read
    // write 0x34 + (oss << 6) to 0xF4
    // wait 4.5ms
    //read 0xF6, 0xF7, 0xF8
    uint8_t oss_value = 0;
    uint8_t control_register_and_start_meas[1] = {static_cast<uint8_t>(Bmp180::Commands::MeasurementControlValues::PRESSURE_OSS0 + (oss_value << 6))};
    buffer_t outbound_message = {
            .bytes = control_register_and_start_meas,
            .size = 1
    };

    register_address = Bmp180::Addresses::DataRegisters::CONTROL_MEASUREMENT;
    if (i2c_send(&_bmp180_i2c_context, &outbound_message, register_address)) {
        std::this_thread::sleep_for(std::chrono::microseconds (4500));
    }
    else {
        std::cout << "failed to command pressure read" << std::endl;
    }

    // read back pressure
    uint8_t uncompensated_pressure[3] = {0};
    buffer_t inbound_message = {
            .bytes = uncompensated_pressure,
            .size = sizeof(uncompensated_pressure)
    };

    register_address = (Bmp180::Addresses::DataRegisters::OUTPUT >> 8) & 0xff;
    i2c_recv(&_bmp180_i2c_context, &inbound_message, register_address);

    _bmp180_long_uncompensated_pressure = (uncompensated_pressure[_device_endian_lsb_index] << 8) | uncompensated_pressure[_device_endian_msb_index];
    _bmp180_short_uncompensated_pressure_xlsb = uncompensated_pressure[2];


}

int Bmp180::_measurement_completed_ok() {

    uint8_t control_measurement_register[1] = {0};
    buffer_t inbound_message = {
            .bytes = control_measurement_register,
            .size = sizeof(control_measurement_register)
    };

    uint8_t register_address = Bmp180::Addresses::DataRegisters::CONTROL_MEASUREMENT;

    int wait_count = 0;
    int wait_limit = 10;
    do {
        i2c_recv(&_bmp180_i2c_context, &inbound_message, register_address);

        bool sco_set = control_measurement_register[0] & Bmp180::BitMasks::ControlRegister::SCO_BIT;

        if(!sco_set) {
            //conversion complete ok (sco is 0)
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::microseconds (50));

        wait_count++;
    }
    while(wait_count < wait_limit);

    return 0;
}
