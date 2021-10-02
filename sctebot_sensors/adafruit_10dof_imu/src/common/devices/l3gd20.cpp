//
// Created by user on 5/23/21.
//

#include "l3gd20.h"
#include "shared_util.h"

int L3gd20Gyro::_init_device(L3gd20Gyro::OutputDataRates_t output_data_rate, L3gd20Gyro::BandwidthCutOff_t bandwidth_cutoff) {

    buffer_t inbound_message;
    buffer_t outbound_message;
    uint8_t register_address;
    uint8_t control_reg[1] = {0};

    // CONTROL REGISTERS 1 to 5
    register_address = L3gd20Gyro::Addresses::CTRL_REG1;

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
     * BOOT             - 0: normal mode (default)
     * FIFO_EN          - 0: FIFO disable (default)
     * StopOnFTH        - 0: FIFO depth is not limited (default)
     * HPen             - 0: HPF disabled (default)
     * IG_Sel1-Sel0     - 00 (default)
     * Out_Sel1-Sel0    - 00 (default)
     */
    control_reg[0] =
            _control_register_1to5_buffer[L3gd20Gyro::Addresses::ControlRegister::CTRL_5];

    //region CTRL_REG4
    /*
     * BDU              - 1: update until MSB and LSB reading
     * BLE              - 0: Data LSB @ lower address (default)
     * FS1-0            - 00: 245 dps (default)
     * IMPen            - 0: level sensitive latched disabled (default)
     * ST2-1            - 00: normal mode (default)
     * SIM              - 1: i2c interface. CS on chip may be tied to high for I2C
     */
    register_address = L3gd20Gyro::Addresses::CTRL_REG4;

    uint8_t full_scale_selection = L3gd20Gyro::BitMasks::ControlRegister4::FS_250;

    switch(full_scale_selection) {
        case L3gd20Gyro::BitMasks::ControlRegister4::FS_250:
            _range_sensitivity = 8.75/1000;
        case L3gd20Gyro::BitMasks::ControlRegister4::FS_500:
            _range_sensitivity = 17.50/1000;
        case L3gd20Gyro::BitMasks::ControlRegister4::FS_2000:
            _range_sensitivity = 70.0/1000;
        default:
            _range_sensitivity = 8.75/1000;
    }

    control_reg[0] =
            _control_register_1to5_buffer[L3gd20Gyro::Addresses::ControlRegister::CTRL_4] |
            L3gd20Gyro::BitMasks::ControlRegister4::BDU |
            full_scale_selection |
            L3gd20Gyro::BitMasks::ControlRegister4::SIM_3WIRE_EN;

    display_register_8bits("CTRL4", _control_register_1to5_buffer[L3gd20Gyro::Addresses::ControlRegister::CTRL_4], "CTRL4", control_reg[0]);

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
    control_reg[0] =
            _control_register_1to5_buffer[L3gd20Gyro::Addresses::ControlRegister::CTRL_3];

    // CTRL_REG2
    /*
     * Default
     */
    control_reg[0] =
        _control_register_1to5_buffer[L3gd20Gyro::Addresses::ControlRegister::CTRL_2];

    //region CTRL_REG1
    /*
     * DR1-0                    - 00: 100Hz (Low ODR is disabled by default)
     * BW1-0                    - 00: 12.5 Hz Cut-Off
     * PD:                      - 1: Normal Mode
     * ZEN:                     - 1: Enabled
     * YEN:                     - 1: Enabled
     * XEN:                     - 1: Enabled
     * i2cset -y 1 0x6b 0x20 0x17
     */
    register_address = L3gd20Gyro::Addresses::CTRL_REG1;

    control_reg[0] =
            //_control_register_1to5_buffer[L3gd20Gyro::Addresses::ControlRegister::CTRL_1] |
            sample_rate_to_register_bitmasks[output_data_rate] |
            bandwidth_cut_off_to_register_bitmasks[bandwidth_cutoff] |
            L3gd20Gyro::BitMasks::ControlRegister1::X_AXIS_ENABLE |
            L3gd20Gyro::BitMasks::ControlRegister1::Y_AXIS_ENABLE |
            L3gd20Gyro::BitMasks::ControlRegister1::Z_AXIS_ENABLE;

    control_reg[0] |= L3gd20Gyro::BitMasks::ControlRegister1::POWER_DOWN_DISABLE;

    //display_register_8bits(_control_register_1to5_buffer[0], control_reg[0]);

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

void L3gd20Gyro::_data_capture_worker() {
    BOOST_LOG_TRIVIAL(debug) << "L3gd20Gyro _data_capture_worker starting";

    std::unique_lock<std::mutex> data_worker_run_thread_lock(this->data_capture_thread_run_mutex);
    BOOST_LOG_TRIVIAL(debug) << "l3gd20 waiting to run...";
    this->data_capture_thread_run_cv.wait(data_worker_run_thread_lock);
    data_worker_run_thread_lock.unlock();

    BOOST_LOG_TRIVIAL(debug) << "l3gd20 running...";

    data_worker_run_thread_lock.lock();
    while(this->run_data_capture_thread) {
        data_worker_run_thread_lock.unlock();

        // maybe make these structs and pass that? less calls?
        float temperature;

        float x_axis;
        float y_axis;
        float z_axis;

        uint8_t gyro_status;

        // update gyro status
        gyro_status = this->_update_gyroscope_status();

        if((gyro_status & L3gd20Gyro::BitMasks::StatusRegister::ZYX_DATA_AVAILABLE) == false) {
            // do nothing
        }
        else {

            this->_update_angular_rate_xyz_axis();
            this->_update_temperature_axis();

            // maybe make these structs and pass that? less calls?

            std::lock_guard<std::mutex> gyro_temp_lock(this->gyroscope_data_mutex);
            {
                temperature = temperature_axis_deg_c;

                x_axis = angular_rate_x_axis_deg_ps;
                y_axis = angular_rate_y_axis_deg_ps;
                z_axis = angular_rate_z_axis_deg_ps;
            }

            this->_host_callback_function(
                    temperature,
                    x_axis, y_axis, z_axis
            );

        }

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_sensor_update_period_ms));

        data_worker_run_thread_lock.lock();
    }

    BOOST_LOG_TRIVIAL(debug) << "L3gd20Gyro _data_capture_worker exiting";
}

void L3gd20Gyro::enable_load_mock_data() {
    _enable_load_mock_data = true;
}

void L3gd20Gyro::_mock_device_emulation() {
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

    /* Real values from sensor */
    debug_x_rate_axis = 0x0130;
    debug_y_rate_axis = 0x0044;
    debug_z_rate_axis = 0x001e;

    debug_temp_axis = 0x01;

    uint8_t register_address;

    device_lock.lock();
    while(this->mock_run_device_thread) {
        device_lock.unlock();

        //region TEMP AXIS
        register_address = L3gd20Gyro::Addresses::Registers::OUT_TEMP;

        uint8_t mock_temperature[1] = {0};
        buffer_t outbound_measurement = {
                .bytes = mock_temperature,
                .size = sizeof(mock_temperature)
        };

        mock_temperature[0] = debug_temp_axis;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region X AXIS
        register_address = L3gd20Gyro::Addresses::Registers::OUT_X_L;

        uint8_t mock_x_axis[2] = {0};
        outbound_measurement = {
                .bytes = mock_x_axis,
                .size = sizeof(mock_x_axis)
        };

        // LSB 0x28
        // MSB 0x29
        mock_x_axis[0] = debug_x_rate_axis & 0xFF;
        mock_x_axis[1] = (debug_x_rate_axis >> 8) & 0xFF;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region Y AXIS
        register_address = L3gd20Gyro::Addresses::Registers::OUT_Y_L;

        uint8_t mock_y_axis[2] = {0};
        outbound_measurement = {
                .bytes = mock_y_axis,
                .size = sizeof(mock_y_axis)
        };

        // LSB 0x2A
        // MSB 0x2B
        mock_y_axis[0] = debug_y_rate_axis & 0xFF;
        mock_y_axis[1] = (debug_y_rate_axis >> 8) & 0xFF;

        i2c_send(&_i2c_device_context, &outbound_measurement, register_address);
        //endregion

        //region Z AXIS
        register_address = L3gd20Gyro::Addresses::Registers::OUT_Z_L;

        uint8_t mock_z_axis[2] = {0};
        outbound_measurement = {
                .bytes = mock_z_axis,
                .size = sizeof(mock_z_axis)
        };

        // LSB 0x2C
        // MSB 0x2D
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

void L3gd20Gyro::_update_temperature_axis() {

    uint8_t register_address;

    uint8_t temperature[1] = {0};
    buffer_t inbound_message = {
            .bytes = temperature,
            .size = sizeof(temperature)
    };

    register_address = L3gd20Gyro::Addresses::Registers::OUT_TEMP;
    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {

        std::lock_guard<std::mutex> gyro_temp_lock(this->gyroscope_data_mutex);
        {
            _temperature_axis_byte = (int8_t)temperature[0];

            // TODO perform scaling conversion
            temperature_axis_deg_c = (float)_temperature_axis_byte;
        }

        display_register_8bits("temp", _temperature_axis_byte, "temp", temperature[0]);
    }
}

void L3gd20Gyro::_update_angular_rate_xyz_axis() {

    uint8_t register_address;

    uint8_t out_xyz_axis[2] = {0};
    buffer_t inbound_message = {
            .bytes = out_xyz_axis,
            .size = sizeof(out_xyz_axis)
    };

    std::lock_guard<std::mutex> gyro_temp_lock(this->gyroscope_data_mutex);
    {
        register_address = L3gd20Gyro::Addresses::Registers::OUT_X_L;
        bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

        if (data_ok) {
            // LSB 0x28
            // MSB 0x29
            _angular_rate_x_axis_bytes = (out_xyz_axis[1] << 8) + out_xyz_axis[0];

            // TODO perform scaling conversion
            angular_rate_x_axis_deg_ps = float(_angular_rate_x_axis_bytes) * _range_sensitivity;

            display_register_16bits("x axis", _angular_rate_x_axis_bytes, "x axis",
                                    (out_xyz_axis[1] << 8) | out_xyz_axis[0]);
        }

        register_address = L3gd20Gyro::Addresses::Registers::OUT_Y_L;
        data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

        if (data_ok) {
            // LSB 0x2A
            // MSB 0x2B
            _angular_rate_y_axis_bytes = (out_xyz_axis[1] << 8) + out_xyz_axis[0];

            // TODO perform scaling conversion
            angular_rate_y_axis_deg_ps = float(_angular_rate_y_axis_bytes) * _range_sensitivity;

            display_register_16bits("y axis", _angular_rate_y_axis_bytes, "y axis",
                                    (out_xyz_axis[1] << 8) | out_xyz_axis[0]);
        }

        register_address = L3gd20Gyro::Addresses::Registers::OUT_Z_L;
        data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

        if (data_ok) {
            // LSB 0x2C
            // MSB 0x2D
            _angular_rate_z_axis_bytes = (out_xyz_axis[1] << 8) + out_xyz_axis[0];

            // TODO perform scaling conversion
            angular_rate_z_axis_deg_ps = float(_angular_rate_z_axis_bytes) * _range_sensitivity;

            display_register_16bits("z axis", _angular_rate_z_axis_bytes, "z axis",
                                    ((out_xyz_axis[1] << 8) + out_xyz_axis[0]));
        }
    }
}

uint8_t L3gd20Gyro::_update_gyroscope_status() {

    uint8_t status_register = 0;

    uint8_t register_address;

    uint8_t gyro_status[1] = {0};
    buffer_t inbound_message = {
            .bytes = gyro_status,
            .size = sizeof(gyro_status)
    };

    register_address = L3gd20Gyro::Addresses::Registers::STATUS_REG;

    bool data_ok = i2c_recv(&_i2c_device_context, &inbound_message, register_address);

    if(data_ok) {

        std::lock_guard<std::mutex> gyroscope_data_lock(this->gyroscope_data_mutex);
        {
            _status_xyz_reg = gyro_status[0];
            status_register = _status_xyz_reg;
        }

        if(status_register & L3gd20Gyro::BitMasks::StatusRegister::ZYX_DATA_AVAILABLE) {

            std::bitset<8> x(status_register);

            bool zyx_or_status = status_register &  L3gd20Gyro::BitMasks::StatusRegister::ZYX_OVERRUN;

            bool z_or = status_register & L3gd20Gyro::BitMasks::StatusRegister::Z_OVERRUN;
            bool y_or = status_register & L3gd20Gyro::BitMasks::StatusRegister::Y_OVERRUN;
            bool x_or = status_register & L3gd20Gyro::BitMasks::StatusRegister::X_OVERRUN;

            bool zyx_da_status = status_register & L3gd20Gyro::BitMasks::StatusRegister::ZYX_DATA_AVAILABLE;

            bool z_da = status_register & L3gd20Gyro::BitMasks::StatusRegister::Z_DATA_AVAILABLE;
            bool y_da = status_register & L3gd20Gyro::BitMasks::StatusRegister::Y_DATA_AVAILABLE;
            bool x_da = status_register & L3gd20Gyro::BitMasks::StatusRegister::X_DATA_AVAILABLE;

            if(z_or | y_or | x_or) {

                BOOST_LOG_TRIVIAL(debug) << "gyro status reg: " << x;

                BOOST_LOG_TRIVIAL(debug) << "zyx_or: " << zyx_or_status << " z_or: " << z_or << " y_or: " << y_or << " x_or: " << x_or;
                BOOST_LOG_TRIVIAL(debug) << "zyx_da: " << zyx_da_status << " z_da: " << z_da << " y_da: " << y_da << " x_da: " << x_da;

            }
        }
    }

    return status_register;
}