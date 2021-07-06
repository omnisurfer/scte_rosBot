//
// Created by user on 4/9/21.
//

#include "bmp180.h"
#include "shared_util.h"

int Bmp180::_init_device() {

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::debug
    );

    buffer_t inbound_message;

    // COEFFICIENT REGISTERS 1 to 11
    uint8_t register_address = (Bmp180::Addresses::CalibrationCoefficients::AC1 >> 8) & 0xff;

    inbound_message = {
            .bytes = _calibration_data_buffer,
            .size = sizeof(_calibration_data_buffer)
    };

    if (i2c_recv(&_i2c_device_context, &inbound_message, register_address)) {

        std::string output_string;
        std::stringstream ss;

        ss << "bmp180 calibration data: " << std::endl;

        for(uint i = 0; i < sizeof(_calibration_data_buffer); ++i) {
            ss << std::hex << std::setfill('0') << std::setw(2) << (int)_calibration_data_buffer[i] << " ";
        }

        output_string = ss.str();

        BOOST_LOG_TRIVIAL(info) << output_string;
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "failed to read bmp180 coefficient registers";
    }

    this->_init_calibration_coefficients(
            (char *) this->_get_calibration_buffer_address(),
            this->_get_calibration_buffer_size()
                                         );

    std::lock_guard<std::mutex> lock(this->data_capture_thread_run_mutex);
    this->data_capture_thread_run_cv.notify_one();
    this->run_data_capture_thread = true;

    return 1;
}

void Bmp180::_data_capture_worker() {
    BOOST_LOG_TRIVIAL(debug) << "_data_capture_worker starting";

    std::unique_lock<std::mutex> data_lock(this->data_capture_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "bmp180 waiting to run...";
    this->data_capture_thread_run_cv.wait(data_lock);
    data_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "bmp180 running...";

    data_lock.lock();
    while(this->run_data_capture_thread) {
        data_lock.unlock();

        this->_request_temperature();

        this->_request_pressure();

        uint16_t long_uncompensated_temperature = this->_get_uncompensated_temperature_count();
        uint16_t long_uncompensated_pressure = this->_get_uncompensated_pressure();
        uint8_t short_uncompensated_pressure_xlsb = this->_get_uncompensated_pressure_xlsb();

        float calculated_temperature = 0.0f;
        this->_calculate_temperature(long_uncompensated_temperature, calculated_temperature);

        float calculated_pressure = 0.0f;
        this->_calculate_pressure(long_uncompensated_pressure, short_uncompensated_pressure_xlsb, calculated_pressure);

        this->_host_callback_function(
                calculated_temperature, calculated_pressure
                );

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_sensor_update_period_ms));

        data_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "_data_capture_worker exiting";
}

void Bmp180::_mock_device_emulation() {
    BOOST_LOG_TRIVIAL(debug) << "_mock_device_emulation starting";

    std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "mock bmp180 waiting to run...";
    this->mock_device_thread_run_cv.wait(device_lock);
    device_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "mock bmp180 running...";

    uint8_t debug_temp_counter = 0x96;
    uint8_t debug_press_counter = 0x07;

    device_lock.lock();
    while(this->mock_run_device_thread) {
        device_lock.unlock();

        uint8_t register_address;

        // read in the register command
        uint8_t measurement_command[1] = {0};

        buffer_t inbound_message = {
                .bytes = measurement_command,
                .size = sizeof(measurement_command)
        };

        register_address = Bmp180::Addresses::DataRegisters::CONTROL_MEASUREMENT;
        i2c_recv(&_i2c_device_context, &inbound_message, register_address);


        //figure out which measurement is being made
        if(measurement_command[0] == Bmp180::Commands::MeasurementControlValues::TEMPERATURE) {
            //std::cout << "Got temp cmd: " << std::hex << int(measurement_command[0]) << std::endl;

            //set the temperature value in memory
            uint8_t mock_temperature[2] = {0};
            buffer_t outbound_measurement = {
                    .bytes = mock_temperature,
                    .size = sizeof(mock_temperature)
            };

            // real temp ~26C
            // MSB 0xF6
            // LSB 0xF7
            mock_temperature[0] = 0x64;
            mock_temperature[1] = debug_temp_counter;

            //debug_temp_counter = (debug_temp_counter + 1)%0x02;

            register_address = Bmp180::Addresses::DataRegisters::OUTPUT_MSB;
            i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        }
        else if (measurement_command[0] == Bmp180::Commands::MeasurementControlValues::PRESSURE_OSS0) {
            //std::cout << "Got press cmd: " << std::hex << int(measurement_command[0]) << std::endl;

            uint8_t mock_pressure[3] = {0};
            buffer_t outbound_measurement = {
                    .bytes = mock_pressure,
                    .size = sizeof(mock_pressure)
            };

            // real pressure ~13 PSI or ~91000 Pa @ ~26C ONLY
            // MSB 0xF6
            // LSB 0xF7
            // XLSB 0xF8
            mock_pressure[0] = 0xa3;
            mock_pressure[1] = debug_press_counter;
            mock_pressure[2] = 0x00;

            debug_press_counter = (debug_press_counter + 1)%0x03;

            register_address = Bmp180::Addresses::DataRegisters::OUTPUT_MSB;
            i2c_send(&_i2c_device_context, &outbound_measurement, register_address);

        }
        else {
            //skip this iteration of the loop since no valid measurement command received
            //std::cout << "Got unknown cmd 0x" << std::hex << int(measurement_command[0]) << std::endl;
            // sleep a little just to prevent busy looping
            std::this_thread::sleep_for(std::chrono::microseconds(500));
            device_lock.lock();
            continue;
        }

        //set the measurement in progress bit
        register_address = Bmp180::Addresses::DataRegisters::CONTROL_MEASUREMENT;
        measurement_command[0] &= Bmp180::BitMasks::ControlRegister::SCO_BIT;
        //std::cout << "Ctrl SCO SET: " << std::bitset<8>(measurement_command[0]) << std::endl;

        buffer_t  outbound_message = {
                .bytes = measurement_command,
                .size = sizeof(measurement_command)
        };

        i2c_send(&_i2c_device_context, &outbound_message, register_address);

        //wait 4.5ms
        std::this_thread::sleep_for(std::chrono::microseconds(4500));

        //clear the measurement in progress bit
        measurement_command[0] &= ~Bmp180::BitMasks::ControlRegister::SCO_BIT;
        //std::cout << "Ctrl SCO CLEAR: " << std::bitset<8>(measurement_command[0]) << std::endl;

        outbound_message = {
                .bytes = measurement_command,
                .size = sizeof(measurement_command)
        };

        i2c_send(&_i2c_device_context, &outbound_message, register_address);

        device_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "_mock_device_emulation exiting";
}

void Bmp180::_request_temperature() {

    uint8_t control_register_and_start_meas[1] = {Bmp180::Commands::MeasurementControlValues ::TEMPERATURE};
    buffer_t outbound_message = {
            .bytes = control_register_and_start_meas,
            .size = sizeof(control_register_and_start_meas)
    };

    uint8_t register_address;

    register_address = Bmp180::Addresses::DataRegisters::CONTROL_MEASUREMENT;
    if (i2c_send(&_i2c_device_context, &outbound_message, register_address)) {

        // wait 4.5ms
        std::this_thread::sleep_for(std::chrono::microseconds (4500));

        if(!this->_measurement_completed_ok()) {
            BOOST_LOG_TRIVIAL(error) << "failed to complete measurement";
            return;
        }
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "failed to command temperature read";
    }

    // read back temperature
    uint8_t uncompensated_temperature[2] = {0};
    buffer_t inbound_message = {
            .bytes = uncompensated_temperature,
            .size = sizeof(uncompensated_temperature)
    };

    register_address = Bmp180::Addresses::DataRegisters::OUTPUT_MSB;
    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        // MSB 0xF6
        // LSB 0xF7
        _long_uncompensated_temperature = (uncompensated_temperature[0] << 8) | uncompensated_temperature[1];
    }

    //std::cout << "temp: " << std::hex << _long_uncompensated_temperature << std::endl;

}

void Bmp180::_request_pressure() {

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
    if (i2c_send(&_i2c_device_context, &outbound_message, register_address)) {

        // wait 4.5ms
        std::this_thread::sleep_for(std::chrono::microseconds (4500));

        if(!this->_measurement_completed_ok()) {
            BOOST_LOG_TRIVIAL(error) << "failed to complete measurement";
            return;
        }
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "failed to command pressure read";
    }

    // read back pressure
    uint8_t uncompensated_pressure[3] = {0};
    buffer_t inbound_message = {
            .bytes = uncompensated_pressure,
            .size = sizeof(uncompensated_pressure)
    };

    register_address = Bmp180::Addresses::DataRegisters::OUTPUT_MSB;
    i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    // MSB 0xF6
    // LSB 0xF7
    // XLSB 0xF8
    _long_uncompensated_pressure = (uncompensated_pressure[0] << 8) + uncompensated_pressure[1];
    _short_uncompensated_pressure_xlsb = uncompensated_pressure[2];

    //std::cout << "press: " << std::hex << _long_uncompensated_pressure << " xlsb " << std::hex << _short_uncompensated_pressure_xlsb << std::endl;

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
        i2c_recv(&_i2c_device_context, &inbound_message, register_address);

        bool sco_set = control_measurement_register[0] & Bmp180::BitMasks::ControlRegister::SCO_BIT;

        if(!sco_set) {
            //conversion complete ok (sco is 0)
            return 1;
        }
        else {
            // do nothing
        }

        std::this_thread::sleep_for(std::chrono::microseconds (500));

        wait_count++;
    }
    while(wait_count < wait_limit);

    return 0;
}