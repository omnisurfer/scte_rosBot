//
// Created by user on 6/21/21.
//
#include <sstream>

#include "lsm303dlhc.h"
#include "shared_util.h"

int Lsm303Dlhc::_init_device() {

    logging::core::get()->set_filter
    (
            logging::trivial::severity >= logging::trivial::debug
    );

    buffer_t inbound_message;
    buffer_t outbound_message;
    uint8_t register_address;
    uint8_t control_reg[1] = {0};

    // CONTROL REGISTERS 1 to 6
    register_address = Lsm303Dlhc::Addresses::CTRL_REG1_A;

    inbound_message = {
            .bytes = _control_register_1to6_buffer,
            .size = sizeof(_control_register_1to6_buffer)
    };

    if(i2c_recv(&_i2c_device_context, &inbound_message, register_address)) {

        std::string output_string;
        std::stringstream ss;

        ss << "lsm303dlhc config: ";

        for(uint i = 0; i < sizeof(_control_register_1to6_buffer); ++i) {
            ss << std::setfill('0') << std::setw(2) << (int)_control_register_1to6_buffer[i] << " ";
        }

        output_string = ss.str();

        BOOST_LOG_TRIVIAL(info) << output_string;
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "failed to read lsm303dlhc control registers";
    }

    //region CTRL_REG1
    /*
     * 1HZ
     */
    register_address = Lsm303Dlhc::Addresses::CTRL_REG1_A;

    control_reg[0] =
            Lsm303Dlhc::BitMasks::ControlRegister1::ODR_1HZ |
            Lsm303Dlhc::BitMasks::Z_AXIS_EN |
            Lsm303Dlhc::BitMasks::Y_AXIS_EN |
            Lsm303Dlhc::BitMasks::X_AXIS_EN;

    display_register_8bits(_control_register_1to6_buffer[0], control_reg[0]);

    outbound_message = {
            .bytes = control_reg,
            .size = sizeof(control_reg)
    };

    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(info) << "CTRL_REG1_A configure OK";
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "CTRL_REG1_A configure failed";
    }
    //endregion

    // CTRL_REG2
    /*
     * DEFAULT
     * HPF @ 0.02Hz
     */

    // CTRL_REG3
    /*
     * DEFAULT
     */

    //region CTRL_REG4
    /*
     * BDU = 1
     * LSB @ Lower address, LITTLE ENDIAN
     * FS Res +/- 2G
     * HiRes disabled
     */
    register_address = Lsm303Dlhc::Addresses::CTRL_REG4_A;

    control_reg[0] =
            _control_register_1to6_buffer[3] |
            Lsm303Dlhc::BitMasks::ControlRegister4::BDU_EN;

    display_register_8bits(_control_register_1to6_buffer[0], control_reg[0]);

    outbound_message = {
            .bytes = control_reg,
            .size = sizeof(control_reg)
    };

    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(info) << "CTRL_REG4_A configure OK";
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "CTRL_REG4_A configure failed";
    }
    //endregion

    // CTRL_REG5
    /*
     * DEFAULT
     */

    // CTRL_REG6
    /*
     * DEFAULT
     */

    // MAGNETIC REGISTERS

    //region CRA_REG_M
    /*
     * Enable temperature
     * Data Output rate = 15Hz (default)
     */
    register_address = Lsm303Dlhc::Addresses::CRA_REG_M;

    inbound_message = {
            .bytes = _cra_reg_m,
            .size = sizeof(_cra_reg_m)
    };

    if(i2c_recv(&_i2c_device_context, &inbound_message, register_address)) {

        std::string output_string;
        std::stringstream ss;

        ss << "lsm303dlhc config: ";

        for(uint i = 0; i < sizeof(_cra_reg_m); ++i) {
            ss << std::hex << std::setfill('0') << std::setw(2) << (int)_cra_reg_m[i] << " ";
        }

        output_string = ss.str();

        BOOST_LOG_TRIVIAL(info) << output_string;
    }

    control_reg[0] =
            _cra_reg_m[0] |
            Lsm303Dlhc::BitMasks::CrARegM::TEMP_EN;

    display_register_8bits(_control_register_1to6_buffer[0], control_reg[0]);

    outbound_message = {
            .bytes = control_reg,
            .size = sizeof(control_reg)
    };

    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(info) << "CRA_REG_M configure OK";
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "CRA_REG_M configure failed";
    }
    //endregion

    // CRB_REG_M
    /*
     * Default gain (TBD)
     */

    //region MR_REG_M
    /*
     * Set to single-conversion mode
     */
    register_address = Lsm303Dlhc::Addresses::MR_REG_M;

    inbound_message = {
            .bytes = _mr_reg_m,
            .size = sizeof(_mr_reg_m)
    };

    if(i2c_recv(&_i2c_device_context, &inbound_message, register_address)) {

        std::string output_string;
        std::stringstream ss;

        ss << "lsm303dlhc config: ";

        for(uint i = 0; i < sizeof(_mr_reg_m); ++i) {
            ss << std::hex << std::setfill('0') << std::setw(2) << (int)_mr_reg_m[i] << " ";
        }

        output_string = ss.str();

        BOOST_LOG_TRIVIAL(info) << output_string;
    }

    control_reg[0] =
            _mr_reg_m[0] |
            Lsm303Dlhc::BitMasks::MrRegM::SINGLE_CONVERSION;

    outbound_message = {
            .bytes = control_reg,
            .size = sizeof(control_reg)
    };

    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(info) << "Reg MR_REG_M configure OK";
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "Reg MR_REG_M configure failed";
    }
    //endregion

    std::lock_guard<std::mutex> lk(this->data_capture_thread_run_mutex);
    this->data_capture_thread_run_cv.notify_one();
    this->run_data_capture_thread = true;

    return 0;
}

void Lsm303Dlhc::_data_capture_worker() {
    BOOST_LOG_TRIVIAL(debug) << "_data_capture_worker starting";

    std::unique_lock<std::mutex> data_lock(this->data_capture_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "lsm303dlhc waiting to run...";
    this->data_capture_thread_run_cv.wait(data_lock);
    data_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "lsm303dlhc running...";

    data_lock.lock();
    while(this->run_data_capture_thread) {
        data_lock.unlock();

        this->_request_temperature_axis();
        this->_request_accelerometer_xyz_axis();
        this->_request_magnetometer_xyz_axis();

        int16_t temperature = this->_get_temperature();

        int16_t x_accel_axis = this->_get_accel_x_axis();
        int16_t y_accel_axis = this->_get_accel_y_axis();
        int16_t z_accel_axis = this->_get_accel_z_axis();

        int16_t x_mag_axis = this->_get_magnetic_x_axis();
        int16_t y_mag_axis = this->_get_magnetic_y_axis();
        int16_t z_mag_axis = this->_get_magnetic_z_axis();

        this->_host_callback_function(
                temperature,
                x_accel_axis, y_accel_axis, z_accel_axis,
                x_mag_axis, y_mag_axis, z_mag_axis
                );

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_sensor_update_period_ms));

        data_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "_data_capture_worker exiting";
}

void Lsm303Dlhc::_mock_device_emulation() {
    BOOST_LOG_TRIVIAL(debug) << "_mock_device_emulation starting";

    std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "mock lsm303dlhc waiting to run...";
    this->mock_device_thread_run_cv.wait(device_lock);
    device_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "mock lsm303dlhc running...";

    uint16_t loop_sleep_microseconds = 10500;

    int16_t debug_temp_axis = -1024;

    int16_t debug_x_accel_axis = 420;
    int16_t debug_y_accel_axis = -420;
    int16_t debug_z_accel_axis = 42;

    int16_t debug_x_mag_axis = 420;
    int16_t debug_y_mag_axis = -420;
    int16_t debug_z_mag_axis = 42;

    uint8_t register_address;

    device_lock.lock();
    while(this->mock_run_device_thread) {
        device_lock.unlock();

        //region TEMP AXIS
        register_address = Lsm303Dlhc::Addresses::Registers::TEMP_OUT_H_M;

        uint8_t mock_temperature[2] = {0};
        buffer_t outbound_measurement = {
                .bytes = mock_temperature,
                .size = sizeof(mock_temperature)
        };

        /*
         * 1024 =  0000 0100 0000 0000
         * -1024 = 1111 1100 0000 0000
         *
         *  0000 0000 0100 0000
         *  0000 0000 1111 1100
         */

        // convert to 12-bit value
        int16_t temp = debug_temp_axis;
        int16_t out_temp_axis = debug_temp_axis << 4;
        mock_temperature[0] = out_temp_axis & 0x00FF;
        mock_temperature[1] = (out_temp_axis & 0xFF00) >> 8;

        display_register_8bits(mock_temperature[0], mock_temperature[1]);
        display_register_16bits(temp, out_temp_axis);

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region ACCEL X AXIS
        register_address = Lsm303Dlhc::Addresses::Registers::OUT_X_L_A;

        uint8_t mock_x_accel[2] = {0};
        outbound_measurement = {
                .bytes = mock_x_accel,
                .size = sizeof(mock_x_accel)
        };

        mock_x_accel[0] = debug_x_accel_axis & 0x00FF;
        mock_x_accel[1] = (debug_x_accel_axis & 0xFF00) >> 8;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region ACCEL Y AXIS
        register_address = Lsm303Dlhc::Addresses::Registers::OUT_Y_L_A;

        uint8_t mock_y_accel[2] = {0};
        outbound_measurement = {
                .bytes = mock_y_accel,
                .size = sizeof(mock_y_accel)
        };

        mock_y_accel[0] = debug_y_accel_axis & 0x00FF;
        mock_y_accel[1] = (debug_y_accel_axis & 0xFF00) >> 8;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region ACCEL Z AXIS
        register_address = Lsm303Dlhc::Addresses::Registers::OUT_Z_L_A;

        uint8_t mock_z_accel[2] = {0};
        outbound_measurement = {
                .bytes = mock_z_accel,
                .size = sizeof(mock_z_accel)
        };

        mock_z_accel[0] = debug_z_accel_axis & 0x00FF;
        mock_z_accel[1] = (debug_z_accel_axis & 0xFF00) >> 8;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region MAG X AXIS
        register_address = Lsm303Dlhc::Addresses::Registers::OUT_X_L_M;

        uint8_t mock_x_mag[2] = {0};
        outbound_measurement = {
                .bytes = mock_x_mag,
                .size = sizeof(mock_x_mag)
        };

        mock_x_mag[0] = debug_x_mag_axis & 0x00FF;
        mock_x_mag[1] = (debug_x_mag_axis & 0xFF00) >> 8;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region MAG Y AXIS
        register_address = Lsm303Dlhc::Addresses::Registers::OUT_Y_L_M;

        uint8_t mock_y_mag[2] = {0};
        outbound_measurement = {
                .bytes = mock_y_mag,
                .size = sizeof(mock_y_mag)
        };

        mock_y_mag[0] = debug_y_mag_axis & 0x00FF;
        mock_y_mag[1] = (debug_y_mag_axis & 0xFF00) >> 8;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region MAG Z AXIS
        register_address = Lsm303Dlhc::Addresses::Registers::OUT_Z_L_M;

        uint8_t mock_z_mag[2] = {0};
        outbound_measurement = {
                .bytes = mock_z_mag,
                .size = sizeof(mock_z_mag)
        };

        mock_z_mag[0] = debug_z_mag_axis & 0x00FF;
        mock_z_mag[1] = (debug_z_mag_axis & 0xFF00) >> 8;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        std::this_thread::sleep_for(std::chrono::microseconds(loop_sleep_microseconds));
        // sleep to help with debug
        std::this_thread::sleep_for(std::chrono::seconds(1));
        device_lock.lock();
    }

}

void Lsm303Dlhc::_request_temperature_axis() {

    uint8_t register_address;

    uint8_t temperature[2] = {0};
    buffer_t inbound_message = {
            .bytes = temperature,
            .size = sizeof(temperature)
    };

    register_address = Lsm303Dlhc::Addresses::Registers::TEMP_OUT_H_M;
    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        // convert back to 16-bit value
        int16_t temp_axis = (temperature[1] << 8) + temperature[0];
        _temperature_axis = temp_axis >> 4;

        display_register_8bits(temperature[0], temperature[1]);
    }
}

void Lsm303Dlhc::_request_accelerometer_xyz_axis() {

    uint8_t register_address;

    uint8_t out_accel_xyz_axis[2] = {0};
    buffer_t inbound_message = {
            .bytes = out_accel_xyz_axis,
            .size = sizeof(out_accel_xyz_axis)
    };

    register_address = Lsm303Dlhc::Addresses::Registers::OUT_X_L_A;
    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        _accelerometer_x_axis = (out_accel_xyz_axis[1] << 8) + out_accel_xyz_axis[0];
    }

    register_address = Lsm303Dlhc::Addresses::Registers::OUT_Y_L_A;
    data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        _accelerometer_y_axis = (out_accel_xyz_axis[1] << 8) + out_accel_xyz_axis[0];
    }

    register_address = Lsm303Dlhc::Addresses::Registers::OUT_Z_L_A;
    data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        _accelerometer_z_axis = (out_accel_xyz_axis[1] << 8) + out_accel_xyz_axis[0];
    }
}

void Lsm303Dlhc::_request_magnetometer_xyz_axis() {

    uint8_t register_address;

    uint8_t out_mag_xyz_axis[2] = {0};
    buffer_t inbound_message = {
            .bytes = out_mag_xyz_axis,
            .size = sizeof(out_mag_xyz_axis)
    };

    register_address = Lsm303Dlhc::Addresses::Registers::OUT_X_L_M;
    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        _magnetometer_x_axis = (out_mag_xyz_axis[1] << 8) + out_mag_xyz_axis[0];
    }

    register_address = Lsm303Dlhc::Addresses::Registers::OUT_Y_L_M;
    data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        _magnetometer_y_axis = (out_mag_xyz_axis[1] << 8) + out_mag_xyz_axis[0];
    }

    register_address = Lsm303Dlhc::Addresses::Registers::OUT_Z_L_M;
    data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        _magnetometer_z_axis = (out_mag_xyz_axis[1] << 8) + out_mag_xyz_axis[0];
    }
}