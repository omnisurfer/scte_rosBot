//
// Created by user on 5/23/21.
//

#include "l3gd20.h"
#include "shared_util.h"

int L3gd20::_init_device() {

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::debug
    );

    buffer_t inbound_message;
    buffer_t outbound_message;
    uint8_t register_address;
    uint8_t control_reg[1] = {0};

    // CONTROL REGISTERS 1 to 5
    register_address = L3gd20::Addresses::CTRL_REG1;

    inbound_message = {
            .bytes = _control_register_1to5_buffer,
            .size = sizeof(_control_register_1to5_buffer)
    };

    if(i2c_recv(&_i2c_device_context, &inbound_message, register_address)) {

        std::string output_string;
        std::stringstream ss;

        ss << "l3gd20 config: ";

        for(uint i = 0; i < sizeof(_control_register_1to5_buffer); ++i) {
            ss << std::hex << std::setfill('0') << std::setw(2) << (int)_control_register_1to5_buffer[i] << " ";
        }

        output_string = ss.str();

        BOOST_LOG_TRIVIAL(info) << output_string;
    }

    // Config CTRL_REG5
    /*
     * DEFAULT
     */

    //region CTRL_REG4
    /*
     * BDU = 1, read update on read
     * 250 dps (DEFAULT)
     */
    register_address = L3gd20::Addresses::CTRL_REG4;

    control_reg[0] =
            _control_register_1to5_buffer[3] |
            L3gd20::BitMasks::ControlRegister4::BDU;

    display_register_bits(_control_register_1to5_buffer[3], control_reg[0]);

    outbound_message = {
            .bytes = control_reg,
            .size = sizeof(control_reg)
    };

    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(debug) << "CTRL_REG4 configure OK";
    }
    else {
        BOOST_LOG_TRIVIAL(debug) << "CTRL_REG4 configure failed";
    }
    //endregion

    // CTRL_REG3
    /*
     * Default
     */

    // CTRL_REG2
    /*
     * Default
     */

    //region CTRL_REG1
    /*
     * Disable power down mode
     * ODR = 95Hz (Default)
     */
    register_address = L3gd20::Addresses::CTRL_REG1;

    control_reg[0] =
            _control_register_1to5_buffer[0] |
            L3gd20::BitMasks::ControlRegister1::X_AXIS_ENABLE |
            L3gd20::BitMasks::ControlRegister1::Y_AXIS_ENABLE |
            L3gd20::BitMasks::ControlRegister1::Z_AXIS_ENABLE;

    control_reg[0] |= L3gd20::BitMasks::ControlRegister1::POWER_DOWN_DISABLE;

    display_register_bits(_control_register_1to5_buffer[0], control_reg[0]);

    outbound_message = {
            .bytes = control_reg,
            .size = sizeof(control_reg)
    };

    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(debug) << "CTRL_REG1 configure OK";
    }
    else {
        BOOST_LOG_TRIVIAL(debug) << "CTRL_REG1 configure failed";
    }
    //endregion

    std::lock_guard<std::mutex> lk(this->data_capture_thread_run_mutex);
    this->data_capture_thread_run_cv.notify_one();
    this->run_data_capture_thread = true;

    return 0;
}

void L3gd20::_data_capture_worker() {
    BOOST_LOG_TRIVIAL(debug) << "_data_capture_worker starting";

    std::unique_lock<std::mutex> data_lock(this->data_capture_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "l3gd20 waiting to run...";
    this->data_capture_thread_run_cv.wait(data_lock);
    data_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "l3gd20 running...";

    data_lock.lock();
    while(this->run_data_capture_thread) {
        data_lock.unlock();

        this->_request_temperature();

        this->_request_angular_rate_xyz_axis();

        // maybe make these structs and pass that? less calls?
        int8_t temperature = this->_get_temperature();
        int16_t x_axis = this->_get_angular_rate_x_axis();
        int16_t y_axis = this->_get_angular_rate_y_axis();
        int16_t z_axis = this->_get_angular_rate_z_axis();

        this->_host_callback_function(temperature, x_axis, y_axis, z_axis);

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_sensor_update_period_ms));

        data_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "_data_capture_worker exiting";
}

void L3gd20::_mock_device_emulation() {
    BOOST_LOG_TRIVIAL(debug) << "_mock_device_emulation starting";

    std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "mock l3gd20 waiting to run...";
    this->mock_device_thread_run_cv.wait(device_lock);
    device_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "mock l3gd20 running...";

    uint16_t loop_sleep_microseconds = 10500;

    int16_t debug_x_rate_axis = 420;
    int16_t debug_y_rate_axis = -420;
    int16_t debug_z_rate_axis = 69;
    int8_t debug_temp_axis = -42;

    uint8_t register_address;

    device_lock.lock();
    while(this->mock_run_device_thread) {
        device_lock.unlock();

        //region TEMP AXIS
        register_address = L3gd20::Addresses::Registers::OUT_TEMP;

        uint8_t mock_temperature[1] = {0};
        buffer_t outbound_measurement = {
                .bytes = mock_temperature,
                .size = sizeof(mock_temperature)
        };

        mock_temperature[0] = debug_temp_axis;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region X AXIS
        register_address = L3gd20::Addresses::Registers::OUT_X_L;

        uint8_t mock_x_axis[2] = {0};
        outbound_measurement = {
                .bytes = mock_x_axis,
                .size = sizeof(mock_x_axis)
        };

        mock_x_axis[0] = debug_x_rate_axis & 0xFF;
        mock_x_axis[1] = (debug_x_rate_axis >> 8) & 0xFF;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region Y AXIS
        register_address = L3gd20::Addresses::Registers::OUT_Y_L;

        uint8_t mock_y_axis[2] = {0};
        outbound_measurement = {
                .bytes = mock_y_axis,
                .size = sizeof(mock_y_axis)
        };

        mock_y_axis[0] = debug_y_rate_axis & 0xFF;
        mock_y_axis[1] = (debug_y_rate_axis >> 8) & 0xFF;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region Z AXIS
        register_address = L3gd20::Addresses::Registers::OUT_Z_L;

        uint8_t mock_z_axis[2] = {0};
        outbound_measurement = {
                .bytes = mock_z_axis,
                .size = sizeof(mock_z_axis)
        };

        mock_z_axis[0] = debug_z_rate_axis & 0xFF;
        mock_z_axis[1] = (debug_z_rate_axis >> 8) & 0xFF;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        std::this_thread::sleep_for(std::chrono::microseconds(loop_sleep_microseconds));
        // sleep to help with debug
        std::this_thread::sleep_for(std::chrono::seconds(1));
        device_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "_mock_device_emulation exiting";
}

void L3gd20::_request_temperature() {

    uint8_t register_address;

    uint8_t temperature[1] = {0};
    buffer_t inbound_message = {
            .bytes = temperature,
            .size = sizeof(temperature)
    };

    register_address = L3gd20::Addresses::Registers::OUT_TEMP;
    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        _temperature_axis = (int8_t)temperature[0];
    }
}

void L3gd20::_request_angular_rate_xyz_axis() {
    uint8_t register_address;

    uint8_t out_xyz_axis[2] = {0};
    buffer_t inbound_message = {
            .bytes = out_xyz_axis,
            .size = sizeof(out_xyz_axis)
    };

    register_address = L3gd20::Addresses::Registers::OUT_X_L;
    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        _angular_rate_x_axis = (out_xyz_axis[1] << 8) + out_xyz_axis[0];
    }

    register_address = L3gd20::Addresses::Registers::OUT_Y_L;
    data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        _angular_rate_y_axis = (out_xyz_axis[1] << 8) + out_xyz_axis[0];
    }

    register_address = L3gd20::Addresses::Registers::OUT_Z_L;
    data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        _angular_rate_z_axis = (out_xyz_axis[1] << 8) + out_xyz_axis[0];
    }
}

int L3gd20::_measurement_completed_ok() {

    return 0;
}