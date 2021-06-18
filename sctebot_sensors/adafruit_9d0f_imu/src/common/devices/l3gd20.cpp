//
// Created by user on 5/23/21.
//

#include <bitset>
#include "l3gd20.h"

int L3gd20::_init_l3gd20() {

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::debug
    );

    buffer_t inbound_message;
    buffer_t outbound_message;
    uint8_t register_address;
    uint8_t control_reg;

    // Read config bytes
    inbound_message = {
            .bytes = _l3gd20_control_register_buffer,
            .size = sizeof(_l3gd20_control_register_buffer)
    };

    register_address = L3gd20::Addresses::CTRL_REG1;
    if(i2c_recv(&_l3gd20_i2c_context, &inbound_message, register_address)) {
        std::cout << "l3gd20 config: " << std::endl;

        for(uint i = 0; i < sizeof(_l3gd20_control_register_buffer); ++i) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)_l3gd20_control_register_buffer[i] << " ";
        }

        std::cout << std::endl;
    }

    // Config CTRL_REG5
    // Default

    // Config CTRL_REG4
    // BDU = 1, read update on read
    control_reg =
            _l3gd20_control_register_buffer[3] |
            L3gd20::BitMasks::ControlRegister4::BDU;

    std::bitset<8> a(_l3gd20_control_register_buffer[3]);
    std::bitset<8> b(control_reg);

    std::cout << "ctrl1b " << a << " ctrl1a " << b << std::endl;

    outbound_message = {
            .bytes = _l3gd20_control_register_buffer,
            .size = sizeof(control_reg)
    };

    register_address = L3gd20::Addresses::CTRL_REG4;
    if(i2c_send(&_l3gd20_i2c_context, &outbound_message, register_address)) {
        std::cout << "sensor commanded to start measuring" << std::endl;
    }
    else {
        std::cout << "failed to command sensor to start measuring" << std::endl;
    }

    // Config CTRL_REG3
    // Default

    // Config CTRL_REG2
    // Default

    // Config CTRL_REG1
    //Disable power down mode
    control_reg =
            _l3gd20_control_register_buffer[0] |
            L3gd20::BitMasks::ControlRegister1::X_AXIS_ENABLE |
            L3gd20::BitMasks::ControlRegister1::Y_AXIS_ENABLE |
            L3gd20::BitMasks::ControlRegister1::Z_AXIS_ENABLE;

    control_reg |= L3gd20::BitMasks::ControlRegister1::POWER_DOWN_DISABLE;

    std::bitset<8> c(_l3gd20_control_register_buffer[0]);
    std::bitset<8> d(control_reg);

    std::cout << "ctrl1b " << c << " ctrl1a " << d << std::endl;

    register_address = L3gd20::Addresses::CTRL_REG1;
    if(i2c_send(&_l3gd20_i2c_context, &outbound_message, register_address)) {
        std::cout << "sensor commanded to start measuring" << std::endl;
    }
    else {
        std::cout << "failed to command sensor to start measuring" << std::endl;
    }

    std::lock_guard<std::mutex> lk(this->l3gd20_data_capture_thread_run_mutex);
    this->l3gd20_data_capture_thread_run_cv.notify_one();
    this->run_l3gd20_data_capture_thread = true;

    return 0;
}

void L3gd20::_l3gd20_data_capture_worker() {
    BOOST_LOG_TRIVIAL(debug) << "_l3gd20_data_capture_worker starting";

    std::unique_lock<std::mutex> data_lock(this->l3gd20_data_capture_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "l3gd20 waiting to run...";
    this->l3gd20_data_capture_thread_run_cv.wait(data_lock);
    data_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "l3gd20 running...";

    data_lock.lock();
    while(this->run_l3gd20_data_capture_thread) {
        data_lock.unlock();

        // DO STUFF

        this->_l3gd20_host_callback_function(0.42, 0.69, 0.420);

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_l3gd20_sensor_update_period_ms));

        data_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "_l3gd20_data_capture_worker exiting";
}

void L3gd20::_mock_l3gd20_device_emulation() {
    BOOST_LOG_TRIVIAL(debug) << "_mock_l3gd20_device_emulation starting";

    std::unique_lock<std::mutex> device_lock(this->mock_l3gd20_device_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "mock l3gd20 waiting to run...";
    this->mock_l3gd20_device_thread_run_cv.wait(device_lock);
    device_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "mock l3gd20 running...";

    uint8_t debug_temp_counter = 0x96;
    uint8_t debug_press_counter = 0x07;

    device_lock.lock();
    while(this->mock_run_l3gd20_device_thread) {
        device_lock.unlock();

        /*
        uint8_t register_address;

        // read in the register command
        uint8_t measurement_command[1] = {0};

        buffer_t inbound_message = {
                .bytes = measurement_command,
                .size = sizeof(measurement_command)
        };

        register_address = L3gd20::Addresses::DataRegisters::CONTROL_MEASUREMENT;
        i2c_recv(&_l3gd20_i2c_context, &inbound_message, register_address);


        //figure out which measurement is being made
        if(measurement_command[0] == L3gd20::Commands::MeasurementControlValues::TEMPERATURE) {
            //std::cout << "Got temp cmd: " << std::hex << int(measurement_command[0]) << std::endl;

            //set the temperature value in memory
            uint8_t mock_temperature[2] = {0};
            buffer_t outbound_measurement = {
                    .bytes = mock_temperature,
                    .size = sizeof(mock_temperature)
            };

            // real temp ~26C
            mock_temperature[0] = 0x64;
            mock_temperature[1] = debug_temp_counter;

            //debug_temp_counter = (debug_temp_counter + 1)%0x02;

            register_address = Bmp180::Addresses::DataRegisters::OUTPUT_MSB;
            i2c_send(&_bmp180_i2c_context, &outbound_measurement, register_address);

        }
        else if (measurement_command[0] == L3gd20::Commands::MeasurementControlValues::PRESSURE_OSS0) {
            //std::cout << "Got press cmd: " << std::hex << int(measurement_command[0]) << std::endl;

            uint8_t mock_pressure[3] = {0};
            buffer_t outbound_measurement = {
                    .bytes = mock_pressure,
                    .size = sizeof(mock_pressure)
            };

            // real pressure ~13 PSI or ~91000 Pa @ ~26C ONLY
            mock_pressure[0] = 0xa3;
            mock_pressure[1] = debug_press_counter;
            mock_pressure[2] = 0x00;

            debug_press_counter = (debug_press_counter + 1)%0x03;

            register_address = Bmp180::Addresses::DataRegisters::OUTPUT_MSB;
            i2c_send(&_bmp180_i2c_context, &outbound_measurement, register_address);

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
        register_address = L3gd20::Addresses::DataRegisters::CONTROL_MEASUREMENT;
        measurement_command[0] &= L3gd20::BitMasks::ControlRegister::SCO_BIT;
        //std::bitset<8> x(measurement_command[0]);
        //std::cout << "Ctrl SCO SET: " << x << std::endl;

        buffer_t  outbound_message = {
                .bytes = measurement_command,
                .size = sizeof(measurement_command)
        };


        i2c_send(&_l3gd20_i2c_context, &outbound_message, register_address);
        */
        //wait 4.5ms
        std::this_thread::sleep_for(std::chrono::microseconds(4500));
        /*
        //clear the measurement in progress bit
        measurement_command[0] &= ~L3gd20::BitMasks::ControlRegister::SCO_BIT;
        //std::bitset<8> y(measurement_command[0]);
        //std::cout << "Ctrl SCO CLEAR: " << y << std::endl;

        outbound_message = {
                .bytes = measurement_command,
                .size = sizeof(measurement_command)
        };

        i2c_send(&_l3gd20_i2c_context, &outbound_message, register_address);
        */

        device_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "_mock_bmp180_device_emulation exiting";
}

int L3gd20::_measurement_completed_ok() {

    return 0;
}