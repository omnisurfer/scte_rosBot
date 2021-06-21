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
    // 250 dps (DEFAULT)
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
    // Disable power down mode
    // ODR = 95Hz (Default)
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

        this->_request_l3gd20_temperature();

        this->_request_l3gd20_xyz_axis();

        // maybe make these structs and pass that? less calls?
        int8_t temperature = this->_get_l3gd20_temperature();
        int16_t x_axis = this->_get_l3gd20_x_axis();
        int16_t y_axis = this->_get_l3gd20_y_axis();
        int16_t z_axis = this->_get_l3gd20_z_axis();

        this->_l3gd20_host_callback_function(temperature, x_axis, y_axis, z_axis);

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

    uint16_t loop_sleep_microseconds = 10500;

    int16_t debug_x_axis = 420;
    int16_t debug_y_axis = -420;
    int16_t debug_z_axis = 69;
    int8_t debug_temp_axis = -42;

    uint8_t register_address;

    device_lock.lock();
    while(this->mock_run_l3gd20_device_thread) {
        device_lock.unlock();

        // TEMP AXIS
        uint8_t mock_temperature[1] = {0};
        buffer_t outbound_measurement = {
                .bytes = mock_temperature,
                .size = sizeof(mock_temperature)
        };

        mock_temperature[0] = debug_temp_axis;

        register_address = L3gd20::Addresses::Registers::OUT_TEMP;
        i2c_send(&_l3gd20_i2c_context, &outbound_measurement, register_address);

        // X - AXIS
        uint8_t mock_x_axis[2] = {0};
        outbound_measurement = {
                .bytes = mock_x_axis,
                .size = sizeof(mock_x_axis)
        };

        mock_x_axis[0] = debug_x_axis & 0xFF;
        mock_x_axis[1] = (debug_x_axis >> 8) & 0xFF;

        //std::cout << "x_l " << mock_x_axis[0] << " x_h " << mock_x_axis[1] << std::endl;

        register_address = L3gd20::Addresses::Registers::OUT_X_L;
        i2c_send(&_l3gd20_i2c_context, &outbound_measurement, register_address);

        // Y - AXIS
        uint8_t mock_y_axis[2] = {0};
        outbound_measurement = {
                .bytes = mock_y_axis,
                .size = sizeof(mock_y_axis)
        };

        mock_y_axis[0] = debug_y_axis & 0xFF;
        mock_y_axis[1] = (debug_y_axis >> 8) & 0xFF;

        register_address = L3gd20::Addresses::Registers::OUT_Y_L;
        i2c_send(&_l3gd20_i2c_context, &outbound_measurement, register_address);

        // Z - AXIS
        uint8_t mock_z_axis[2] = {0};
        outbound_measurement = {
                .bytes = mock_z_axis,
                .size = sizeof(mock_z_axis)
        };

        mock_z_axis[0] = debug_z_axis & 0xFF;
        mock_z_axis[1] = (debug_z_axis >> 8) & 0xFF;

        register_address = L3gd20::Addresses::Registers::OUT_Z_L;
        i2c_send(&_l3gd20_i2c_context, &outbound_measurement, register_address);

        std::this_thread::sleep_for(std::chrono::microseconds(loop_sleep_microseconds));
        // sleep to help with debug
        std::this_thread::sleep_for(std::chrono::seconds(1));
        device_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "_mock_bmp180_device_emulation exiting";
}

void L3gd20::_request_l3gd20_temperature() {

    uint8_t register_address;

    uint8_t temperature[1] = {0};
    buffer_t inbound_message = {
            .bytes = temperature,
            .size = sizeof(temperature)
    };

    register_address = L3gd20::Addresses::Registers::OUT_TEMP;
    bool data_ok = i2c_recv(&_l3gd20_i2c_context, &inbound_message, register_address);

    if(data_ok) {
        _l3gd20_temperature = (int8_t)temperature[0];
    }
}

void L3gd20::_request_l3gd20_xyz_axis() {
    uint8_t register_address;

    uint8_t out_xyz_axis[2] = {0};
    buffer_t inbound_message = {
            .bytes = out_xyz_axis,
            .size = sizeof(out_xyz_axis)
    };

    register_address = L3gd20::Addresses::Registers::OUT_X_L;
    bool data_ok = i2c_recv(&_l3gd20_i2c_context, &inbound_message, register_address);

    if(data_ok) {
        _l3gd20_x_axis = (out_xyz_axis[1] << 8) + out_xyz_axis[0];
    }

    register_address = L3gd20::Addresses::Registers::OUT_Y_L;
    data_ok = i2c_recv(&_l3gd20_i2c_context, &inbound_message, register_address);

    if(data_ok) {
        _l3gd20_y_axis = (out_xyz_axis[1] << 8) + out_xyz_axis[0];
    }

    register_address = L3gd20::Addresses::Registers::OUT_Z_L;
    data_ok = i2c_recv(&_l3gd20_i2c_context, &inbound_message, register_address);

    if(data_ok) {
        _l3gd20_z_axis = (out_xyz_axis[1] << 8) + out_xyz_axis[0];
    }
}

int L3gd20::_measurement_completed_ok() {

    return 0;
}