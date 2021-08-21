//
// Created by user on 4/9/21.
//

#ifndef L3GD20_H
#define L3GD20_H

#include <iostream>
#include <iomanip>
#include <cstring>
#include <thread>
#include <condition_variable>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace logging = boost::log;

#include "i2c_linux.h"

/*
 * TODO CONVERT DRIVER TO L3GD20H! I WAS READING THE DATA SHEET FOR THE NON-H DEVICE TYPE.
 * THIS EXPLAINS THE D7 WHO_AM_I INSTEAD OF D4
 */
class L3gd20Gyro {

    // 1101 0101 0xD5 (Read address)
    // 1101 0100 0xD4 (Write address)

private:

    class Addresses {

    public:
        typedef enum Registers_t {
            WHO_AM_I = 0x0F,
            CTRL_REG1 = 0x20,
            CTRL_REG2 = 0x21,
            CTRL_REG3 = 0x22,
            CTRL_REG4 = 0x23,
            CTRL_REG5 = 0x24,
            REFERENCE = 0x25,
            OUT_TEMP = 0x26,
            STATUS_REG = 0x27,
            OUT_X_L = 0x28,
            OUT_X_H = 0x29,
            OUT_Y_L = 0x2A,
            OUT_Y_H = 0x2B,
            OUT_Z_L = 0x2C,
            OUT_Z_H = 0x2D,
            FIFO_CTRL_REG = 0x2E,
            FIFO_SRC_REG = 0x2F,
            INT1_CFG = 0x30,
            INT1_SRC = 0x31,
            INT1_TSH_XH = 0x32,
            INT1_TSH_XL = 0x33,
            INT1_TSH_YH = 0x34,
            INT1_TSH_YL = 0x35,
            INT1_TSH_ZH = 0x36,
            INT1_TSH_ZL = 0x37,
            INT1_DURATION = 0x38,
            LOW_ODR = 0x39
        } Registers;

        typedef enum ControlRegister_t {
            CTRL_1 = 0,
            CTRL_2 = 1,
            CTRL_3 = 2,
            CTRL_4 = 3,
            CTRL_5 = 4
        } ControlRegister;
    };

    class MagicNumbers {
    public:
        typedef enum WhoAmI_t {
            WHO_AM_I = 0xD7
        }WhoAmI;
    };

    class BitMasks {
    public:
        typedef enum ControlRegister1_t {
            ODR_95 = 0b00000000,
            ODR_190 = 0b01000000,
            ODR_380 = 0b10000000,
            ODR_760 = 0b11000000,
            BW_CO1 = 0b00000000,
            BW_CO2 = 0b00010000,
            BW_CO3 = 0b00100000,
            BW_CO4 = 0b00110000,
            POWER_DOWN_DISABLE = (1 << 3),
            Z_AXIS_ENABLE = (1 << 2),
            X_AXIS_ENABLE = (1 << 1),
            Y_AXIS_ENABLE = (1 << 0)
        } ControlRegister1;

        typedef enum ControlRegister2_t {
            NORMAL_MODE_RESET = 0b00000000,
            REF_SIGNAL_FILTER = 0b00010000,
            NORMAL_MODE = 0b00100000,
            AUTORESET_ON_INT = 0b00110000,
            HPFCO0 = 0b00000000,
            HPFCO1 = 0b00000001,
            HPFCO2 = 0b00000010,
            HPFCO3 = 0b00000011,
            HPFCO4 = 0b00000100,
            HPFCO5 = 0b00000101,
            HPFCO6 = 0b00000110,
            HPFCO7 = 0b00000111,
            HPFCO8 = 0b00001000,
            HPFCO9 = 0b00001001
        } ControlRegister2;

        typedef enum ControlRegister3_t {
            INT1_EN = (1 << 7),
            INT1_BOOT_STATUS = (1 << 6),
            INT1_ACTIVE = (1 << 5),
            PP_OD = (1 << 4),
            INT2_DRDY = (1 << 3),
            INT2_WTM = (1 << 2),
            INT2_ORUN = (1 << 1),
            INT2_EMPTY = (1 << 0)
        } ControlRegister3;

        typedef enum ControlRegister4_t {
            BDU = (1 << 7),
            BLE = (1 << 6),
            FS_250 = 0b00000000,
            FS_500 = 0b00010000,
            FS_2000 = 0b00100000,
            SIM_3WIRE_EN = (1 << 0)

        } ControlRegister4;

        typedef enum ControlRegister5_t {
            BOOT = (1 << 7),
            FIFO_EN = (1 << 6),
            HPEN = (1 << 4)
        } ControlRegister5;

        typedef enum StatusRegister_t {
            ZYX_OVERRUN = (1 << 7),
            Z_OVERRUN = (1 << 6),
            Y_OVERRUN = (1 << 5),
            X_OVERRUN = (1 << 4),
            ZYX_DATA_AVAILABLE = (1 << 3),
            Z_DATA_AVAILABLE = (1 << 2),
            Y_DATA_AVAILABLE = (1 << 1),
            X_DATA_AVAILABLE = (1 << 0)
        } StatusRegister;

        typedef enum FifoControlRegister_t {
            BYPASS_MODE = 0b00000000,
            FIFO_MODE = 0b00100000,
            STREAM_MODE = 0b01000000,
            STREAM_TO_FIFO = 0b01100000,
            BYPASS_TO_STREAM = 0b10000000
        } FifoControlRegister;

        typedef enum FifoSourceRegister_t {
            WATERMARK_STATUS = (1 << 7),
            OVERRUN_STATUS = (1 << 6),
            FIFO_EMPTY = (1 << 5),
            FIFO_STORED_DATA = 0b00000000
        } FifoSourceRegister;

        typedef enum Int1Config_t {
            AND_OR_COMBINATION_INTERRUPTS = (1 << 7),
            LATCH_INTERRUPT_REQUEST = (1 << 6),
            ENABLE_INT_GENERATION_ON_Z_HIGH = (1 << 5),
            ENABLE_INT_GENERATION_ON_Z_LOW = (1 << 4),
            ENABLE_INT_GENERATION_ON_Y_HIGH = (1 << 3),
            ENABLE_INT_GENERATION_ON_Y_LOW = (1 << 2),
            ENABLE_INT_GENERATION_ON_X_HIGH = (1 << 1),
            ENABLE_INT_GENERATION_ON_X_LOW = (1 << 0)
        } Int1Config;

        typedef enum Int1Source_t {
            INT_ACTIVE = (1 << 7),
            Z_HIGH_EVENT = (1 << 6),
            Z_LOW_EVENT = (1 << 5),
            Y_HIGH_EVENT = (1 << 4),
            Y_LOW_EVENT = (1 << 3),
            X_HIGH_EVENT = (1 << 2),
            X_LOW_EVENT = (1 << 1)
        } Int1Source;

        typedef enum Int1Duration_t {
            WAIT_ENABLE = (1 << 7)
        } Int1Duration;
    };

    int _i2c_bus_number{};
    int _i2c_device_address{};
    int _sensor_update_period_ms{};
    std::string _device_name;

    context_t _i2c_device_context{};

    bool _enable_load_mock_data = false;

    uint8_t _control_register_1to5_buffer[5] = {0};
    uint8_t _status_xyz_reg{0};

    int8_t _temperature_axis_byte{0};
    float temperature_axis_deg_c{0};

    int16_t _angular_rate_x_axis_bytes{0};
    int16_t _angular_rate_y_axis_bytes{0};
    int16_t _angular_rate_z_axis_bytes{0};
    float angular_rate_x_axis_deg_ps{0};
    float angular_rate_y_axis_deg_ps{0};
    float angular_rate_z_axis_deg_ps{0};

    typedef struct l3gd20GyroData_s {
        float x;
        float y;
        float z;
    } l3gd20GyroData;

    float _range_sensitivity = 0.0;

    bool run_data_capture_thread = false;
    std::condition_variable data_capture_thread_run_cv;
    std::mutex data_capture_thread_run_mutex;
    std::thread data_capture_thread;

    typedef void (*host_callback_function)(
            float temperature,
            float x_axis, float y_axis, float z_axis
            );
    host_callback_function _host_callback_function{};

    bool mock_run_device_thread = false;
    std::condition_variable mock_device_thread_run_cv;
    std::mutex mock_device_thread_run_mutex;
    std::thread mock_device_thread;

    int _init_device();

    int _connect_to_device() {

        /*
         * setup the i2c context and connect
         */
        _i2c_device_context = {0};
        if(!i2c_dev_open(&_i2c_device_context, _i2c_bus_number, _i2c_device_address)) {
            BOOST_LOG_TRIVIAL(error) << "failed to open device";
            return 0;
        }

        if(!i2c_is_connected(&_i2c_device_context)) {
            BOOST_LOG_TRIVIAL(error) << "failed to connect to device";
            return 0;
        }

        if(_enable_load_mock_data) {
            _mock_load_data();
        }

        // try read whoami
        uint8_t chip_id[1] = {0};
        buffer_t inbound_message = {
                .bytes = chip_id,
                .size = sizeof(chip_id)
        };

        uint8_t register_address;
        register_address = L3gd20Gyro::Addresses::Registers::WHO_AM_I;
        i2c_recv(&_i2c_device_context, &inbound_message, register_address);

        if(chip_id[0] != L3gd20Gyro::MagicNumbers::WhoAmI::WHO_AM_I) {
            BOOST_LOG_TRIVIAL(error) << "failed to read device WHO_AM_I register";
            return 0;
        }
        return 1;
    }

    int _close_device() {

        i2c_dev_close(&_i2c_device_context, _i2c_bus_number);

        return 0;
    }

    int _mock_load_data() {
        /*
         * @77 bmp180 (?)
         * @6b l3gd20 (?)
         * @1e lsm303dlhc (?)
         */

        /* Sensors connected
        0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
        00:          -- -- -- -- -- -- -- -- -- -- -- -- --
        10: -- -- -- -- -- -- -- -- -- 19 -- -- -- -- 1e --
        20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        60: -- -- -- -- -- -- -- -- -- -- -- 6b -- -- -- --
        70: 70 -- -- -- -- -- -- 77
        */

        /* Sensors removed
             0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
        00:          -- -- -- -- -- -- -- -- -- -- -- -- --
        10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        40: 40 -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
        70: 70 -- -- -- -- -- -- --
         */

        /* @6b Idle?
             0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f    0123456789abcdef
        00: a0 d6 7c 01 66 aa a5 82 ff c0 00 00 00 00 00 d7    ??|?f???.?.....?
        10: 43 a0 dc 40 83 32 16 22 a2 bb c5 83 88 38 36 09    C??@?2?"?????86?
        20: 07 00 00 00 00 00 04 00 18 ff ed 00 f3 fe 00 20    ?.....?.?.?.??.
        30: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        40: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        50: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        60: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        70: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        80: a0 d6 7c 01 66 aa a5 82 ff c0 00 00 00 00 00 d7    ??|?f???.?.....?
        90: 43 a0 dc 40 83 32 16 22 a2 bb c5 83 88 38 36 09    C??@?2?"?????86?
        a0: 07 00 00 00 00 00 04 00 18 ff ed 00 f3 fe 00 20    ?.....?.?.?.??.
        b0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        c0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        d0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        e0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        f0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
         */

        uint16_t _mock_device_memory[] = {

                0xa0d6, 0x7c01, 0x66aa, 0xa582, 0xffc0, 0x0000, 0x0000, 0x00d7,
                0x43a0, 0xdc40, 0x8332, 0x1622, 0xa2bb, 0xc583, 0x8838, 0x3609,
                0x0700, 0x0000, 0x0000, 0x0400, 0x18ff, 0xed00, 0xf3fe, 0x0020,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0xa0d6, 0x7c01, 0x66aa, 0xa582, 0xffc0, 0x0000, 0x0000, 0x00d7,
                0x43a0, 0xdc40, 0x8332, 0x1622, 0xa2bb, 0xc583, 0x8838, 0x3609,
                0x0700, 0x0000, 0x0000, 0x0400, 0x18ff, 0xed00, 0xf3fe, 0x0020,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000
        };

        uint8_t send_buffer[sizeof(_mock_device_memory)];
        std::memset(&send_buffer, 0, sizeof(send_buffer));

        /*
         * the mock memory buffer is big endian because it was copied straight from the device.
         * this is why I need to swap bytes manually to store as little endian in memory.
         */

        for(uint i = 0; i < sizeof(_mock_device_memory)/sizeof(*_mock_device_memory); i++) {
            send_buffer[i + (i * 1)] = (_mock_device_memory[i] >> 8) & 0xff;
            send_buffer[i + (i * 1) + 1] = (_mock_device_memory[i]) & 0xff;
        }

        buffer_t outbound_message = {
                .bytes = send_buffer,
                .size = sizeof(send_buffer)
        };

        int8_t register_address = 0x00;

        if (i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
            BOOST_LOG_TRIVIAL(debug) << "loaded mock data for device OK";
            return 0;
        }
        else {
            BOOST_LOG_TRIVIAL(debug) << "loaded mock data for device FAILED";
            return -1;
        }
    }

    void _data_capture_worker();

    void _mock_device_emulation();

    void _request_temperature_axis();

    void _request_angular_rate_xyz_axis();

    int8_t _get_temperature() const {
        return _temperature_axis_byte;
    }

    float _get_angular_rate_x_axis() const {
        return _angular_rate_x_axis_bytes * _range_sensitivity;
    }

    float _get_angular_rate_y_axis() const {
        return _angular_rate_y_axis_bytes * _range_sensitivity;
    }

    float _get_angular_rate_z_axis() const {
        return _angular_rate_z_axis_bytes * _range_sensitivity;
    }

    int _measurement_completed_ok();

public:

    L3gd20Gyro() = default;

    ~L3gd20Gyro() {

        this->_close_device();

        this->run_data_capture_thread = false;

        std::unique_lock<std::mutex> data_lock(this->data_capture_thread_run_mutex);
        this->data_capture_thread_run_cv.notify_one();
        data_lock.unlock();

        if(data_capture_thread.joinable()) {
            data_capture_thread.join();
        }

        this->mock_run_device_thread = false;

        std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
        this->mock_device_thread_run_cv.notify_one();
        device_lock.unlock();

        if(mock_device_thread.joinable()) {
            mock_device_thread.join();
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

        data_capture_thread = std::thread(&L3gd20Gyro::_data_capture_worker, this);

        return 0;
    }

    int connect_to_device() {
        int status = 1;

        status &= this->_connect_to_device();

        return status;
    }

    int init_device() {
        this->_init_device();

        return 1;
    }

    void enable_load_mock_data();

    int mock_run_device_emulation() {

        mock_device_thread = std::thread(&L3gd20Gyro::_mock_device_emulation, this);

        // wait a little bit for the thread to get started
        std::this_thread::sleep_for(std::chrono::milliseconds (10));

        std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
        this->mock_device_thread_run_cv.notify_one();
        this->mock_run_device_thread = true;
        device_lock.unlock();

        return 1;
    }

};

#endif //L3GD20_H
