//
// Created by user on 6/21/21.
//
#include <sstream>

#include "lsm303dlhc.h"
#include "shared_util.h"

// region Accelerometer

int Lsm303DlhcAccelerometer::_init_device() {

    logging::core::get()->set_filter
    (
            logging::trivial::severity >= logging::trivial::debug
    );

    buffer_t inbound_message;
    buffer_t outbound_message;
    uint8_t register_address;
    uint8_t control_reg[1] = {0};

    // CONTROL REGISTERS 1 to 6
    register_address = Lsm303DlhcAccelerometer::Addresses::CTRL_REG1_A;

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

    /* REGISTER SETUP ORDER MAY MATTER - Or the way I am executing the config is screwing up the device */
    /* Issue may have been enable temp sensor?? */

    // CTRL_REG6
    /*
     * DEFAULT
     */

    // CTRL_REG5
    /*
     * DEFAULT
     */

    //region CTRL_REG4
    /*
     * BDU = 0
     * LSB @ Lower address, LITTLE ENDIAN
     * FS Res +/- 2G
     * HiRes disabled
     * i2cset -y 1 0x19 0x23 0x80
     */
    register_address = Lsm303DlhcAccelerometer::Addresses::CTRL_REG4_A;

    uint8_t accel_sensitivity_config = Lsm303DlhcAccelerometer::BitMasks::ControlRegister4::FS_2G_SEL;

    /*
    FS bit set to 00 1
    FS bit set to 01 2
    FS bit set to 10 4
    FS bit set to 11 12
     */

    switch(accel_sensitivity_config) {
        case Lsm303DlhcAccelerometer::BitMasks::ControlRegister4::FS_2G_SEL:
            _linear_acceleration_sensitivity = (1.0/1000);
            break;
        case Lsm303DlhcAccelerometer::BitMasks::ControlRegister4::FS_4G_SEL:
            _linear_acceleration_sensitivity = 2.0/1000;
            break;
        case Lsm303DlhcAccelerometer::BitMasks::ControlRegister4::FS_8G_SEL:
            _linear_acceleration_sensitivity = 4.0/1000;
            break;
        case Lsm303DlhcAccelerometer::BitMasks::ControlRegister4::FS_16G_SEL:
            _linear_acceleration_sensitivity = 12.0/1000;
            break;
        default:
            _linear_acceleration_sensitivity = 1.0/1000;
    }

    control_reg[0] =
            accel_sensitivity_config |
            Lsm303DlhcAccelerometer::BitMasks::ControlRegister4::HI_RES_OUT_EN |
            Lsm303DlhcAccelerometer::BitMasks::ControlRegister4::BDU_EN;

    display_register_8bits("REG4A", _control_register_1to6_buffer[0], "REG4A", control_reg[0]);

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

    // CTRL_REG3
    /*
     * DEFAULT
     */

    // CTRL_REG2
    /*
     * DEFAULT
     * HPF @ 0.02Hz
     */

    //region CTRL_REG1
    /*
     * 1HZ
     * i2cset -y 1 0x19 0x20 0x17
     */
    register_address = Lsm303DlhcAccelerometer::Addresses::CTRL_REG1_A;

    control_reg[0] =
            Lsm303DlhcAccelerometer::BitMasks::ControlRegister1::ODR_1HZ |
            Lsm303DlhcAccelerometer::BitMasks::Z_AXIS_EN |
            Lsm303DlhcAccelerometer::BitMasks::Y_AXIS_EN |
            Lsm303DlhcAccelerometer::BitMasks::X_AXIS_EN;

    display_register_8bits("REG1A", _control_register_1to6_buffer[0], "REG1A", control_reg[0]);

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

    std::lock_guard<std::mutex> lk(this->data_capture_thread_run_mutex);
    this->data_capture_thread_run_cv.notify_one();
    this->run_data_capture_thread = true;

    return 0;
}

int Lsm303DlhcAccelerometer::_connect_to_device() {

    _i2c_device_context = {0};
    if(!i2c_dev_open(&_i2c_device_context, _i2c_bus_number, _i2c_device_address)) {
        BOOST_LOG_TRIVIAL(error) << "failed to open device";
        return 0;
    }

    if(!i2c_is_connected(&_i2c_device_context)) {
        BOOST_LOG_TRIVIAL(error) << "failed to connect to device\n";
        return 0;
    }

    if(_enable_load_mock_data) {
        _mock_load_accel_data();
    }

    return 1;
}

int Lsm303DlhcAccelerometer::_close_device() {
    i2c_dev_close(&_i2c_device_context, _i2c_bus_number);

    return 0;
}

void Lsm303DlhcAccelerometer::_data_capture_worker() {
    BOOST_LOG_TRIVIAL(debug) << "_data_capture_worker starting";

    std::unique_lock<std::mutex> data_lock(this->data_capture_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "lsm303dlhc accelerometer waiting to run...";
    this->data_capture_thread_run_cv.wait(data_lock);
    data_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "lsm303dlhc accelerometer running...";

    data_lock.lock();
    while(this->run_data_capture_thread) {
        data_lock.unlock();

        this->_update_accelerometer_status();

        this->_update_accelerometer_xyz_axis();

        float x_accel_axis = accelerometer_x_axis_g;
        float y_accel_axis = accelerometer_y_axis_g;
        float z_accel_axis = accelerometer_z_axis_g;

        this->_host_callback_function(
                x_accel_axis, y_accel_axis, z_accel_axis
                );

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_sensor_update_period_ms));

        data_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "accelerometer _data_capture_worker exiting";
}

void Lsm303DlhcAccelerometer::_mock_device_emulation() {
    BOOST_LOG_TRIVIAL(debug) << "_mock_device_emulation starting";

    std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "mock lsm303dlhc waiting to run...";
    this->mock_device_thread_run_cv.wait(device_lock);
    device_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "mock lsm303dlhc running...";

    uint16_t loop_sleep_microseconds = 10500;

    int16_t debug_x_accel_axis = 420;
    int16_t debug_y_accel_axis = -420;
    int16_t debug_z_accel_axis = 690;

    /* Real values from sensor */
    debug_x_accel_axis = 0xff70;
    debug_y_accel_axis = 0x3f40;
    debug_z_accel_axis = 0x000f;

    uint8_t register_address;

    device_lock.lock();
    while(this->mock_run_device_thread) {
        device_lock.unlock();

        //region ACCEL X AXIS
        register_address = Lsm303DlhcAccelerometer::Addresses::Registers::OUT_X_L_A;

        uint8_t mock_x_accel[2] = {0};
        buffer_t outbound_measurement = {
                .bytes = mock_x_accel,
                .size = sizeof(mock_x_accel)
        };

        // X_L_A 0x28
        mock_x_accel[0] = debug_x_accel_axis & 0x00FF;
        // X_H_A 0x29
        mock_x_accel[1] = (debug_x_accel_axis & 0xFF00) >> 8;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region ACCEL Y AXIS
        register_address = Lsm303DlhcAccelerometer::Addresses::Registers::OUT_Y_L_A;

        uint8_t mock_y_accel[2] = {0};
        outbound_measurement = {
                .bytes = mock_y_accel,
                .size = sizeof(mock_y_accel)
        };

        // Y_L_A 0x2A
        mock_y_accel[0] = debug_y_accel_axis & 0x00FF;
        // Y_H_A 0x2B
        mock_y_accel[1] = (debug_y_accel_axis & 0xFF00) >> 8;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region ACCEL Z AXIS
        register_address = Lsm303DlhcAccelerometer::Addresses::Registers::OUT_Z_L_A;

        uint8_t mock_z_accel[2] = {0};
        outbound_measurement = {
                .bytes = mock_z_accel,
                .size = sizeof(mock_z_accel)
        };

        // Y_L_A 0x2C
        mock_z_accel[0] = debug_z_accel_axis & 0x00FF;
        // Y_L_A 0x2D
        mock_z_accel[1] = (debug_z_accel_axis & 0xFF00) >> 8;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        std::this_thread::sleep_for(std::chrono::microseconds(loop_sleep_microseconds));
        // sleep to help with debug
        std::this_thread::sleep_for(std::chrono::seconds(1));
        device_lock.lock();
    }

}

void Lsm303DlhcAccelerometer::enable_load_mock_data() {
    _enable_load_mock_data = true;
}

void Lsm303DlhcAccelerometer::_update_accelerometer_xyz_axis() {

    uint8_t register_address;

    uint8_t out_accel_xyz_axis[2] = {0};
    buffer_t inbound_message = {
            .bytes = out_accel_xyz_axis,
            .size = sizeof(out_accel_xyz_axis)
    };

    /*
     * i2cget -y 1 0x1e 0x28 (H)
     * i2cget -y 1 0x1e 0x29 (L)
    */

    register_address = Lsm303DlhcAccelerometer::Addresses::Registers::OUT_X_L_A;
    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        // X_L_A 0x28
        // X_H_A 0x29
        _raw_accelerometer_x_axis = (out_accel_xyz_axis[1] << 8) + out_accel_xyz_axis[0];

        // Shift to 12-bit value as it seems the DLHC modem may not actually support 16-bit
        // https://github.com/pololu/lsm303-arduino/blob/master/LSM303.cpp
        _raw_accelerometer_x_axis = _raw_accelerometer_x_axis >> 4;

        accelerometer_x_axis_g = float(_raw_accelerometer_x_axis) * _linear_acceleration_sensitivity;
    }

    register_address = Lsm303DlhcAccelerometer::Addresses::Registers::OUT_Y_L_A;
    data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        // Y_L_A 0x2A
        // Y_H_A 0x2B
        _raw_accelerometer_y_axis = (out_accel_xyz_axis[1] << 8) + out_accel_xyz_axis[0];

        // Shift to 12-bit value as it seems the DLHC modem may not actually support 16-bit
        _raw_accelerometer_y_axis = _raw_accelerometer_y_axis >> 4;

        accelerometer_y_axis_g = float(_raw_accelerometer_y_axis) * _linear_acceleration_sensitivity;
    }

    register_address = Lsm303DlhcAccelerometer::Addresses::Registers::OUT_Z_L_A;
    data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        // Z_L_A 0x2C
        // Z_H_A 0x2D
        _raw_accelerometer_z_axis = (out_accel_xyz_axis[1] << 8) + out_accel_xyz_axis[0];

        // Shift to 12-bit value as it seems the DLHC modem may not actually support 16-bit
        _raw_accelerometer_z_axis = _raw_accelerometer_z_axis >> 4;

        accelerometer_z_axis_g = float(_raw_accelerometer_z_axis) * _linear_acceleration_sensitivity;
    }
}

void Lsm303DlhcAccelerometer::_update_accelerometer_status() {

    uint8_t register_address;

    uint8_t accel_status[1] = {0};
    buffer_t inbound_message = {
            .bytes = accel_status,
            .size = sizeof(accel_status)
    };

    register_address = Lsm303DlhcAccelerometer::Addresses::Registers::STATUS_REG_A;
    bool status_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(status_ok) {
        _status_register = accel_status[0];

        std::bitset<8> x(_status_register);
        std::cout << "status reg: " << x << std::endl;
    }
}

// endregion

// region Magnetometer

int Lsm303DlhcMagnetometer::_init_device() {

    logging::core::get()->set_filter
    (
            logging::trivial::severity >= logging::trivial::debug
            );

    buffer_t inbound_message;
    buffer_t outbound_message;
    uint8_t register_address;
    uint8_t control_reg[1] = {0};

    //region CRA_REG_M
    /*
     * Enable temperature_deg_c
     * Data Output rate = 15Hz (default)
     * i2cset -y 1 0x19 0x00 0x90
     */
    register_address = Lsm303DlhcMagnetometer::Addresses::CRA_REG_M;

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
            Lsm303DlhcMagnetometer::BitMasks::CrARegM::TEMP_EN |
            Lsm303DlhcMagnetometer::BitMasks::CrARegM::DATA_OUTPUT_RATE_7P5_HZ;

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

    // region CRB_REG_M
    /*
     * +/- 1.3 guass
     */
    register_address = Lsm303DlhcMagnetometer::Addresses::CRB_REG_M;

    inbound_message = {
            .bytes = _crb_reg_m,
            .size = sizeof(_crb_reg_m)
    };

    if(i2c_recv(&_i2c_device_context, &inbound_message, register_address)) {

        std::string output_string;
        std::stringstream ss;

        ss << "lsm303dlhc config: ";

        for(uint i = 0; i < sizeof(_crb_reg_m); ++i) {
            ss << std::hex << std::setfill('0') << std::setw(2) << (int)_crb_reg_m[i] << " ";
        }

        output_string = ss.str();

        BOOST_LOG_TRIVIAL(info) << output_string;
    }

    uint8_t mag_gain_config = Lsm303DlhcMagnetometer::BitMasks::CrBRegM::GAIN_CONFIG_0;

    /*
        GN bits set to 001 (X,Y) 1100
        GN bits set to 001 (Z) 980

        GN bits set to 010 (X,Y) 855
        GN bits set to 010 (Z) 760

        GN bits set to 011 (X,Y) 670
        GN bits set to 011 (Z) 600

        GN bits set to 100 (X,Y) 450
        GN bits set to 100 (Z) 400

        GN bits set to 101 (X,Y) 400
        GN bits set to 101 (Z) 355

        GN bits set to 110 (X,Y) 330
        GN bits set to 110 (Z) 295

        GN bits set to 111(2) (X,Y) 230
        GN bits set to 111(2) (Z) 205

     */

    switch(mag_gain_config) {
        case Lsm303DlhcMagnetometer::BitMasks::CrBRegM::GAIN_CONFIG_0:
            _mag_xy_gain_config = 1.0/1100;
            _mag_z_gain_config = 1.0/980;
            break;
        case Lsm303DlhcMagnetometer::BitMasks::CrBRegM::GAIN_CONFIG_1:
            _mag_xy_gain_config = 1.0/855;
            _mag_z_gain_config = 1.0/760;
            break;
        case Lsm303DlhcMagnetometer::BitMasks::CrBRegM::GAIN_CONFIG_2:
            _mag_xy_gain_config = 1.0/670;
            _mag_z_gain_config = 1.0/600;
            break;
        case Lsm303DlhcMagnetometer::BitMasks::CrBRegM::GAIN_CONFIG_3:
            _mag_xy_gain_config = 1.0/450;
            _mag_z_gain_config = 1.0/450;
            break;
        case Lsm303DlhcMagnetometer::BitMasks::CrBRegM::GAIN_CONFIG_4:
            _mag_xy_gain_config = 1.0/400;
            _mag_z_gain_config = 1.0/355;
            break;
        case Lsm303DlhcMagnetometer::BitMasks::CrBRegM::GAIN_CONFIG_5:
            _mag_xy_gain_config = 1.0/330;
            _mag_z_gain_config = 1.0/295;
            break;
        case Lsm303DlhcMagnetometer::BitMasks::CrBRegM::GAIN_CONFIG_6:
            _mag_xy_gain_config = 1.0/230;
            _mag_z_gain_config = 1.0/205;
            break;
        default:
            _mag_xy_gain_config = 1.0/1100;
            _mag_z_gain_config = 1.0/980;
    }

    control_reg[0] =
            //_crb_reg_m[0] |
            mag_gain_config;

    outbound_message = {
            .bytes = control_reg,
            .size = sizeof(control_reg)
    };

    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(info) << "CRB_REG_M configure OK";
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "CRB_REG_M configure failed";
    }

    // endregion

    //region MR_REG_M
    /*
     * Set to single-conversion mode
     * i2cset -y 1 0x19 0x02 0x01
     */
    register_address = Lsm303DlhcMagnetometer::Addresses::MR_REG_M;

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
            //_mr_reg_m[0] |
            Lsm303DlhcMagnetometer::BitMasks::MrRegM::CONTINUOUS_CONVERSION;

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

int Lsm303DlhcMagnetometer::_connect_to_device() {

    _i2c_device_context = {0};
    if(!i2c_dev_open(&_i2c_device_context, _i2c_bus_number, _i2c_device_address)) {
        BOOST_LOG_TRIVIAL(error) << "failed to open device";
        return 0;
    }

    if(!i2c_is_connected(&_i2c_device_context)) {
        BOOST_LOG_TRIVIAL(error) << "failed to connect to device\n";
        return 0;
    }

    if(_enable_load_mock_data) {
        _mock_load_mag_data();
    }

    // TBD

    return 1;
}

int Lsm303DlhcMagnetometer::_close_device() {
    i2c_dev_close(&_i2c_device_context, _i2c_bus_number);

    return 0;
}

void Lsm303DlhcMagnetometer::_data_capture_worker() {
    BOOST_LOG_TRIVIAL(debug) << "_data_capture_worker starting";

    std::unique_lock<std::mutex> data_lock(this->data_capture_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "lsm303dlhc magnetometer waiting to run...";
    this->data_capture_thread_run_cv.wait(data_lock);
    data_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "lsm303dlhc magnetometer running...";

    data_lock.lock();
    while(this->run_data_capture_thread) {
        data_lock.unlock();

        this->_request_temperature_axis();
        float temperature_deg_c = temperature_axis_degrees_c;

        this->_request_magnetometer_xyz_axis();
        float x_mag_axis = magnetometer_x_axis_gauss;
        float y_mag_axis = magnetometer_y_axis_gauss;
        float z_mag_axis = magnetometer_z_axis_gauss;

        this->_host_callback_function(
                temperature_deg_c,
                x_mag_axis, y_mag_axis, z_mag_axis
                );

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_sensor_update_period_ms));

        data_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "magnetometer _data_capture_worker exiting";
}

void Lsm303DlhcMagnetometer::enable_load_mock_data() {
    _enable_load_mock_data = true;
}

void Lsm303DlhcMagnetometer::_mock_device_emulation() {
    BOOST_LOG_TRIVIAL(debug) << "_mock_device_emulation starting";

    std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "mock lsm303dlhc waiting to run...";
    this->mock_device_thread_run_cv.wait(device_lock);
    device_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "mock lsm303dlhc running...";

    uint16_t loop_sleep_microseconds = 10500;

    int16_t debug_temp_axis = -1024;

    int16_t debug_x_mag_axis = 420;
    int16_t debug_y_mag_axis = -420;
    int16_t debug_z_mag_axis = 690;

    /* Real sensor values - NOTE Byte order is Big Endian:0xMSB-LSB*/
    debug_x_mag_axis = 0x0129;
    debug_z_mag_axis = 0xfe95;
    debug_y_mag_axis = 0xffbd;

    debug_temp_axis = 0x04c0;

    uint8_t register_address;

    device_lock.lock();
    while(this->mock_run_device_thread) {
        device_lock.unlock();

        //region TEMP AXIS
        register_address = Lsm303DlhcMagnetometer::Addresses::Registers::TEMP_OUT_H_M;

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

#if 0
        // convert to 12-bit value - skip this, conversion only occurs on read
        int16_t temp = debug_temp_axis;
        int16_t out_temp_axis = debug_temp_axis << 4;
#endif

        // T_H 0x31
        mock_temperature[0] = (debug_temp_axis & 0xFF00) >> 8;
        // T_L 0x32
        mock_temperature[1] = debug_temp_axis & 0x00FF;

        display_register_8bits("mock temp 0", mock_temperature[0], "mock temp 1", mock_temperature[1]);

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region MAG X AXIS
        register_address = Lsm303DlhcMagnetometer::Addresses::Registers::OUT_X_H_M;

        uint8_t mock_x_mag[2] = {0};
        outbound_measurement = {
                .bytes = mock_x_mag,
                .size = sizeof(mock_x_mag)
        };

        // X_H_M 0x03
        mock_x_mag[0] = (debug_x_mag_axis & 0xFF00) >> 8;
        // X_L_M 0x04
        mock_x_mag[1] = debug_x_mag_axis & 0x00FF;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region MAG Y AXIS
        register_address = Lsm303DlhcMagnetometer::Addresses::Registers::OUT_Y_H_M;

        uint8_t mock_y_mag[2] = {0};
        outbound_measurement = {
                .bytes = mock_y_mag,
                .size = sizeof(mock_y_mag)
        };

        // X_H_M 0x05
        mock_y_mag[0] = (debug_y_mag_axis & 0xFF00) >> 8;
        // X_L_M 0x06
        mock_y_mag[1] = debug_y_mag_axis & 0x00FF;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region MAG Z AXIS
        register_address = Lsm303DlhcMagnetometer::Addresses::Registers::OUT_Z_H_M;

        uint8_t mock_z_mag[2] = {0};
        outbound_measurement = {
                .bytes = mock_z_mag,
                .size = sizeof(mock_z_mag)
        };

        // X_H_M 0x07
        mock_z_mag[0] = (debug_z_mag_axis & 0xFF00) >> 8;
        // X_L_M 0x08
        mock_z_mag[1] = debug_z_mag_axis & 0x00FF;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        std::this_thread::sleep_for(std::chrono::microseconds(loop_sleep_microseconds));
        // sleep to help with debug
        std::this_thread::sleep_for(std::chrono::seconds(1));
        device_lock.lock();
    }

}

void Lsm303DlhcMagnetometer::_request_temperature_axis() {

    uint8_t register_address;

    uint8_t temperature[2] = {0};
    buffer_t inbound_message = {
            .bytes = temperature,
            .size = sizeof(temperature)
    };

    /*
     * i2cget -y 1 0x1e 0x31 (H)
     * i2cget -y 1 0x1e 0x32 (L)
     */

    register_address = Lsm303DlhcMagnetometer::Addresses::Registers::TEMP_OUT_H_M;
    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        // convert to 16-bit value
        // T_H_M 0x31
        // T_L_M 0x32
        int16_t temp_axis = (temperature[0] << 8) + temperature[1];
        _temperature_axis_bytes = temp_axis >> 4;

        // per https://electronics.stackexchange.com/questions/219032/how-to-determine-temperature-with-lsm303dlhc

        // Reference temperature_deg_c is 25C, count = 0
        // +8 counts = 1C worth of change
        // +80 counts = ~10C worth of change

        float transfer_funciton = (1.0/8.0);
        float sensor_offset = 20.0;

        temperature_axis_degrees_c = sensor_offset + (_temperature_axis_bytes * transfer_funciton);

        display_register_8bits("temp0", temperature[0], "temp1", temperature[1]);
    }
}

void Lsm303DlhcMagnetometer::_request_magnetometer_xyz_axis() {

    uint8_t register_address;

    uint8_t out_mag_xyz_axis[2] = {0};
    buffer_t inbound_message = {
            .bytes = out_mag_xyz_axis,
            .size = sizeof(out_mag_xyz_axis)
    };

    register_address = Lsm303DlhcMagnetometer::Addresses::Registers::OUT_X_H_M;
    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        // X_H_M 0x03
        // X_L_M 0x04
        _magnetometer_x_axis_bytes = (out_mag_xyz_axis[0] << 8) + out_mag_xyz_axis[1];

        magnetometer_x_axis_gauss = float(_magnetometer_x_axis_bytes) * _mag_xy_gain_config;
    }

    register_address = Lsm303DlhcMagnetometer::Addresses::Registers::OUT_Y_H_M;
    data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        // Y_H_M 0x07
        // Y_L_M 0x08
        _magnetometer_y_axis_bytes = (out_mag_xyz_axis[0] << 8) + out_mag_xyz_axis[1];

        magnetometer_y_axis_gauss = float(_magnetometer_y_axis_bytes) * _mag_xy_gain_config;
    }

    register_address = Lsm303DlhcMagnetometer::Addresses::Registers::OUT_Z_H_M;
    data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {
        // Z_H_M 0x05
        // Z_L_M 0x06
        _magnetometer_z_axis_bytes = (out_mag_xyz_axis[0] << 8) + out_mag_xyz_axis[1];

        magnetometer_z_axis_gauss = float(_magnetometer_z_axis_bytes) * _mag_z_gain_config;
    }
}

// endregion