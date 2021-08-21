//
// Created by user on 4/9/21.
//

#ifndef LSM303DLHC_H
#define LSM303DLHC_H

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

class Lsm303DlhcAccelerometer {

private:

    class Addresses {

    public:
        typedef enum Registers_t {
            CTRL_REG1_A	= 0x20,
            CTRL_REG2_A	= 0x21,
            CTRL_REG3_A	= 0x22,
            CTRL_REG4_A	= 0x23,
            CTRL_REG5_A	= 0x24,
            CTRL_REG6_A	= 0x25,
            REFERENCE_A	= 0x26,
            STATUS_REG_A = 0x27,
            OUT_X_L_A = 0x28,
            OUT_X_H_A = 0x29,
            OUT_Y_L_A = 0x2A,
            OUT_Y_H_A = 0x2B,
            OUT_Z_L_A = 0x2C,
            OUT_Z_H_A = 0x2D,
            FIFO_CTRL_REG_A	= 0x2E,
            FIFO_SRC_REG_A = 0x2F,
            INT1_CFG_A = 0x30,
            INT1_SOURCE_A = 0x31,
            INT1_THS_A	= 0x32,
            INT1_DURATION_A	= 0x33,
            INT2_CFG_A = 0x34,
            INT2_SOURCE_A = 0x35,
            INT2_THS_A	= 0x36,
            INT2_DURATION_A	= 0x37,
            CLICK_CFG_A	= 0x38,
            CLICK_SRC_A	= 0x39,
            CLICK_THS_A	= 0x3A,
            TIME_LIMIT_A = 0x3B,
            TIME_LATENCY_A = 0x3C,
            TIME_WINDOW_A = 0x3D

        } Registers;
    };

    class MagicNumbers {

    };

    class BitMasks {

    public:
        typedef enum ControlRegister1_t {
            ODR_POWER_DOWN = 0b00000000,
            ODR_1HZ = 0b00010000,
            ODR_10HZ = 0b00100000,
            ODR_25HZ = 0b00110000,
            ODR_50HZ = 0b01000000,
            ODR_100HZ = 0b01010000,
            ODR_200HZ = 0b01100000,
            ODR_400HZ = 0b01110000,
            ODR_1620HZ = 0b10000000,
            ODR_1344HZ = 0b10010000,
            LOW_POWER_EN = (1 << 3),
            Z_AXIS_EN = (1 << 2),
            Y_AXIS_EN = (1 << 1),
            X_AXIS_EN = (1 << 0)
        } ControlRegister1;

        typedef enum ControlRegister2_t {
            HPFM_NORMAL_RESET = 0b00000000,
            HPFM_REF_SIG_FOR_FILTER = 0b01000000,
            HPFM_NORMAL_MODE = 0b10000000,
            HPFM_AUTORESET_INT_EVENT = 0b11000000,
            FILTERED_DATA_SELECTION = (1 << 3),
            HPF_EN_CLICK = (1 << 2),
            HPF_EN_AOI_INT2 = (1 << 1),
            HPF_EN_AOI_INT1 = (1 << 0)
        } ControlRegister2;

        typedef enum ControlRegister3_t {
            I1_CLICK_INT1 = (1 << 7),
            I1_AOI_INT1 = (1 << 6),
            I1_AOI_INT2 = (1 << 5),
            I1_DRDY1_INT1 = (1 << 4),
            I1_DRDY2_INT1 = (1 << 3),
            I1_WTM_INT1 = (1 << 2),
            I1_OVERRUN_INT1 = (1 << 1)
        } ControlRegister3;

        typedef enum ControlRegister4_t {
            BDU_EN = (1 << 7),
            BE_SEL = (1 << 6),
            FS_2G_SEL = 0b00000000,
            FS_4G_SEL = 0b00100000,
            FS_8G_SEL = 0b01000000,
            FS_16G_SEL = 0b00110000,
            HI_RES_OUT_EN = (1 << 3),
            I2C_SEL = (1 << 0)
        } ControlRegister4;

        typedef enum ControlRegister5_t {
            REBOOT_MEMORY = (1 << 7),
            FIFO_EN = (1 << 6),
            LIR_INT1_REQ = (1 << 3),
            D4D_INT1_EN = (1 << 2),
            LIR_INT2_REQ = (1 << 1),
            D4D_INT2_EN = (1 << 0)
        } ControlRegister5;

        typedef enum ControlRegister6_t {
            I2_CLICK_EN = (1 << 7),
            I2_INT1_EN = (1 << 6),
            I2_INT2_EN = (1 << 5),
            REBOOT_MEMORY_PAD2 = (1 << 4),
            ACTIVE_FUNCTION_PAD2 = (1 << 3),
            INT_ACT_HIGH = (1 << 1)
        } ControlRegister6;

        typedef enum StatusRegisterA_t {
            ZXY_OVERRUN = (1 << 7),
            Z_OVERRUN = (1 << 6),
            Y_OVERRUN = (1 << 5),
            X_OVERRUN = (1 << 4),
            ZXY_DATA_AVAILABLE = (1 << 3),
            Z_DATA_AVAILABLE = (1 << 2),
            Y_DATA_AVAILABLE = (1 << 1),
            X_DATA_AVAILABLE = (1 << 0)
        } StatusRegisterA;

        typedef enum FifoControlRegisterA_t {
            BYPASS_MODE = 0b00000000,
            FIFO_MODE = 0b01000000,
            STREAM_MODE = 0b10000000,
            TRIGGER_MODE = 0b11000000,
            TRIGGER_SEL_INT2 = (1 << 5),
            TRIGGER_SEL_INT1 = (0 << 5)
        } FifoControlRegisterA;

        typedef enum FifoSourceRegisterA_t {
            WTM = (1 << 7),
            OVRN_FIFO = (1 << 6),
            EMPTY = (1 << 5)
        } FifoSourceRegisterA;

        typedef enum Int1ConfigA_t {
            AND_OR_INT_1 = (1 << 7),
            SIXD_DETECTION_EN_1 = (1 << 6),
            Z_HIGH_INT_EN_1 = (1 << 5),
            Z_LOW_INT_EN_1 = (1 << 4),
            Y_HIGH_INT_EN_1 = (1 << 3),
            Y_LOW_INT_EN_1 = (1 << 2),
            X_HIGH_INT_EN_1 = (1 << 1),
            X_LOW_INT_EN_1 = (1 << 0)
        } Int1ConfigA;

        typedef enum Int1SourceA_t {
            INT_ACTIVE_1 = (1 << 6),
            Z_HIGH_1 = (1 << 5),
            Z_LOW_1 = (1 << 4),
            Y_HIGH_1 = (1 << 3),
            Y_LOW_1 = (1 << 2),
            X_HIGH_1 = (1 << 1),
            X_LOW_1 = (1 << 0)
        } Int1SourceA;

        typedef enum Int2ConfigA_t {
            AND_OR_INT_2 = (1 << 7),
            SIXD_DETECTION_EN_2 = (1 << 6),
            Z_HIGH_INT_EN_2 = (1 << 5),
            Z_LOW_INT_EN_2 = (1 << 4),
            Y_HIGH_INT_EN_2 = (1 << 3),
            Y_LOW_INT_EN_2 = (1 << 2),
            X_HIGH_INT_EN_2 = (1 << 1),
            X_LOW_INT_EN_2 = (1 << 0)
        } Int2ConfigA;

        typedef enum Int2SourceA_t {
            INT_ACTIVE_2 = (1 << 6),
            Z_HIGH_2 = (1 << 5),
            Z_LOW_2 = (1 << 4),
            Y_HIGH_2 = (1 << 3),
            Y_LOW_2 = (1 << 2),
            X_HIGH_2 = (1 << 1),
            X_LOW_2 = (1 << 0)
        } Int2SourceA;

        typedef enum ClickCfgA_t {
            ZD_EN = (1 << 5),
            ZS_EN = (1 << 4),
            YD_EN = (1 << 3),
            YS_EN = (1 << 2),
            XD_EN = (1 << 1),
            XS_EN = (1 << 0)
        } ClickCfgA;

        typedef enum ClickSrcA_t {
            IA = (1 << 6),
            DCLICK_EN = (1 << 5),
            SLICK_EN = (1 << 4),
            SIGN = (1 << 3),
            Z_DETECT = (1 << 2),
            Y_DETECT = (1 << 1),
            X_DETECT = (1 << 0)
        } ClickSrcA;

        typedef enum CrARegM_t {
            TEMP_EN = (1 << 7),
            DATA_OUTPUT_RATE_0P75_HZ = 0b00000000,
            DATA_OUTPUT_RATE_1P5_HZ = 0b00000100,
            DATA_OUTPUT_RATE_3P0_HZ = 0b00001000,
            DATA_OUTPUT_RATE_7P5_HZ = 0b00001100,

            DATA_OUTPUT_RATE_15P0_HZ = 0b00010000,
            DATA_OUTPUT_RATE_30P0_HZ = 0b00010100,
            DATA_OUTPUT_RATE_75P0_HZ = 0b00011000,
            DATA_OUTPUT_RATE_200P0_HZ = 0b00011100
        } CrARegM;

        typedef enum CrBRegM_t {
            GAIN_CONFIG_0 = 0b00100000,
            GAIN_CONFIG_1 = 0b01000000,
            GAIN_CONFIG_2 = 0b01100000,
            GAIN_CONFIG_3 = 0b10000000,
            GAIN_CONFIG_4 = 0b10100000,
            GAIN_CONFIG_5 = 0b11000000,
            GAIN_CONFIG_6 = 0b11100000
        } CrBRegM;

        typedef enum MrRegM_t {
            CONTINUOUS_CONVERSION = 0b00000000,
            SINGLE_CONVERSION = 0b00000001,
            SLEEP_MODE = 0b00000010
        } MrRegM;

        typedef enum SrRegM_t {
            LOCK = (1 << 1),
            DATA_READY = (1 << 0)
        } SrRegM;
    };

    int _i2c_bus_number{};
    int _i2c_device_address{};
    int _sensor_update_period_ms{};
    std::string _device_name;

    context_t _i2c_device_context{};

    bool _enable_load_mock_data = false;

    uint8_t _control_register_1to6_buffer[6] = {0};

    int16_t _accelerometer_x_axis{0};
    int16_t _accelerometer_y_axis{0};
    int16_t _accelerometer_z_axis{0};

    bool run_data_capture_thread = false;
    std::condition_variable data_capture_thread_run_cv;
    std::mutex data_capture_thread_run_mutex;
    std::thread data_capture_thread;

    typedef void (*host_callback_function)(
            int x_accel_axis, int y_accel_axis, int z_accel_axis
            );

    host_callback_function _host_callback_function{};

    bool mock_run_device_thread = false;
    std::condition_variable mock_device_thread_run_cv;
    std::mutex mock_device_thread_run_mutex;
    std::thread mock_device_thread;

    int _init_device();

    int _connect_to_device();

    int _close_device();

    int _mock_load_accel_data() {

        /* @0x19 - Idle?
             0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f    0123456789abcdef
        00: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 33    ...............3
        10: b2 bb b4 22 01 32 e3 9d 23 38 70 68 e0 20 80 00    ???"?2??#8ph? ?.
        20: 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 20    ?..............
        30: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        40: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        50: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        60: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        70: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        80: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 33    ...............3
        90: b2 bb b4 22 01 32 e3 9d 23 38 70 68 e0 20 80 00    ???"?2??#8ph? ?.
        a0: 07 00 00 00 00 00 00 00 00 00 00 00 00 00 00 20    ?..............
        b0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        c0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        d0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        e0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        f0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
         */

        uint16_t _mock_device_memory[] = {
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0033,
                0xb2bb, 0xb422, 0x0132, 0xe39d, 0x2338, 0x7068, 0xe020, 0x8000,
                0x0700, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0033,
                0xb2bb, 0xb422, 0x0132, 0xe39d, 0x2338, 0x7068, 0xe020, 0x8000,
                0x0700, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0020,
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

            //std::cout << "i " << i << std::endl;
            send_buffer[i + (i * 1)] = (_mock_device_memory[i] >> 8) & 0xff;
            send_buffer[i + (i * 1) + 1] = (_mock_device_memory[i]) & 0xff;
        }

        buffer_t outbound_message = {
                .bytes = send_buffer,
                .size = sizeof(send_buffer)
        };

        int8_t register_address = 0x00;

        if (i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
            BOOST_LOG_TRIVIAL(debug) << "sent mock calibration data to device OK";
            return 0;
        }

        return -1;
    }

    void _data_capture_worker();

    void _mock_device_emulation();

    void _request_accelerometer_xyz_axis();

    int16_t _get_accel_x_axis() const {
        return _accelerometer_x_axis;
    }

    int16_t _get_accel_y_axis() const {
        return _accelerometer_y_axis;
    }

    int16_t _get_accel_z_axis() const {
        return _accelerometer_z_axis;
    }

public:

    Lsm303DlhcAccelerometer() = default;

    ~Lsm303DlhcAccelerometer() {

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
            host_callback_function function_pointer
            ) {
        _i2c_bus_number = bus_number;
        _i2c_device_address = device_address;
        _sensor_update_period_ms = update_period_ms;
        _device_name = std::move(device_name);

        _host_callback_function = function_pointer;

        data_capture_thread = std::thread(&Lsm303DlhcAccelerometer::_data_capture_worker, this);

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

        mock_device_thread = std::thread(&Lsm303DlhcAccelerometer::_mock_device_emulation, this);

        // wait a little bit for the thread to get started
        std::this_thread::sleep_for(std::chrono::milliseconds (10));

        std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
        this->mock_device_thread_run_cv.notify_one();
        this->mock_run_device_thread = true;
        device_lock.unlock();

        return 1;
    }

};

class Lsm303DlhcMagnetometer {

private:

    class Addresses {

    public:
        typedef enum Registers_t {
            CRA_REG_M = 0x00,
            CRB_REG_M = 0x01,
            MR_REG_M = 0x02,
            OUT_X_H_M = 0x03,
            OUT_X_L_M = 0x04,
            OUT_Z_H_M = 0x05,
            OUT_Z_L_M = 0x06,
            OUT_Y_H_M = 0x07,
            OUT_Y_L_M = 0x08,
            SR_REG_Mg = 0x09,
            IRA_REG_M = 0x0A,
            IRB_REG_M = 0x0B,
            IRC_REG_M = 0x0C,
            TEMP_OUT_H_M = 0x31,
            TEMP_OUT_L_M = 0x32

        } Registers;
    };

    class MagicNumbers {

    };

    class BitMasks {

    public:
        typedef enum CrARegM_t {
            TEMP_EN = (1 << 7),
            DATA_OUTPUT_RATE_0P75_HZ = 0b00000000,
            DATA_OUTPUT_RATE_1P5_HZ = 0b00000100,
            DATA_OUTPUT_RATE_3P0_HZ = 0b00001000,
            DATA_OUTPUT_RATE_7P5_HZ = 0b00001100,

            DATA_OUTPUT_RATE_15P0_HZ = 0b00010000,
            DATA_OUTPUT_RATE_30P0_HZ = 0b00010100,
            DATA_OUTPUT_RATE_75P0_HZ = 0b00011000,
            DATA_OUTPUT_RATE_200P0_HZ = 0b00011100
        } CrARegM;

        typedef enum CrBRegM_t {
            GAIN_CONFIG_0 = 0b00100000,
            GAIN_CONFIG_1 = 0b01000000,
            GAIN_CONFIG_2 = 0b01100000,
            GAIN_CONFIG_3 = 0b10000000,
            GAIN_CONFIG_4 = 0b10100000,
            GAIN_CONFIG_5 = 0b11000000,
            GAIN_CONFIG_6 = 0b11100000
        } CrBRegM;

        typedef enum MrRegM_t {
            CONTINUOUS_CONVERSION = 0b00000000,
            SINGLE_CONVERSION = 0b00000001,
            SLEEP_MODE = 0b00000010
        } MrRegM;

        typedef enum SrRegM_t {
            LOCK = (1 << 1),
            DATA_READY = (1 << 0)
        } SrRegM;
    };

    int _i2c_bus_number{};
    int _i2c_device_address{};
    int _sensor_update_period_ms{};
    std::string _device_name;

    context_t _i2c_device_context{};

    bool _enable_load_mock_data = false;

    uint8_t _cra_reg_m[1] = {0};
    uint8_t _crb_reg_m[1] = {0};
    uint8_t _mr_reg_m[1] = {0};

    int16_t _magnetometer_x_axis{0};
    int16_t _magnetometer_y_axis{0};
    int16_t _magnetometer_z_axis{0};

    int16_t _temperature_axis{0};

    bool run_data_capture_thread = false;
    std::condition_variable data_capture_thread_run_cv;
    std::mutex data_capture_thread_run_mutex;
    std::thread data_capture_thread;

    typedef void (*host_callback_function)(
            int temperature,
            int x_mag_axis, int y_mag_axis, int z_mag_axis
            );

    host_callback_function _host_callback_function{};

    bool mock_run_device_thread = false;
    std::condition_variable mock_device_thread_run_cv;
    std::mutex mock_device_thread_run_mutex;
    std::thread mock_device_thread;

    int _init_device();

    int _connect_to_device();

    int _close_device();

    int _mock_load_mag_data() {

        /* @0x1e - Idle?
             0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f    0123456789abcdef
        00: 10 20 03 01 17 fe c5 ff cf 03 48 34 33 00 00 3c    ? ?????.??H43..<
        10: 00 00 00 00 00 00 00 00 00 00 00 83 9c 85 e8 10    ...........?????
        20: 00 00 00 00 00 00 00 00 10 00 00 00 00 00 00 00    ........?.......
        30: 00 00 00 1c 84 65 55 00 a0 00 07 00 00 00 00 00    ...??eU.?.?.....
        40: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        50: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        60: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        70: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        80: 10 20 03 01 17 fe c5 ff cf 03 48 34 33 00 00 3c    ? ?????.??H43..<
        90: 00 00 00 00 00 00 00 00 00 00 00 83 9c 85 e8 10    ...........?????
        a0: 00 00 00 00 00 00 00 00 10 00 00 00 00 00 00 00    ........?.......
        b0: 00 00 00 1c 84 65 55 00 a0 00 07 00 00 00 00 00    ...??eU.?.?.....
        c0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        d0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        e0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        f0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
         */

        uint16_t _mock_device_memory[] = {
                0x1020, 0x0301, 0x17fe, 0xc5ff, 0xcf03, 0x4834, 0x3300, 0x003c,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0083, 0x9c85, 0xe810,
                0x0000, 0x0000, 0x0000, 0x0000, 0x1000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x001c, 0x8465, 0x5500, 0xa000, 0x0700, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x1020, 0x0301, 0x17fe, 0xc5ff, 0xcf03, 0x4834, 0x3300, 0x003c,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0083, 0x9c85, 0xe810,
                0x0000, 0x0000, 0x0000, 0x0000, 0x1000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x001c, 0x8465, 0x5500, 0xa000, 0x0700, 0x0000, 0x0000,
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

            //std::cout << "i " << i << std::endl;
            send_buffer[i + (i * 1)] = (_mock_device_memory[i] >> 8) & 0xff;
            send_buffer[i + (i * 1) + 1] = (_mock_device_memory[i]) & 0xff;
        }

        buffer_t outbound_message = {
                .bytes = send_buffer,
                .size = sizeof(send_buffer)
        };

        int8_t register_address = 0x00;

        if (i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
            BOOST_LOG_TRIVIAL(debug) << "sent mock calibration data to device OK";
            return 0;
        }

        return -1;
    }

    void _data_capture_worker();

    void _mock_device_emulation();

    void _request_temperature_axis();

    void _request_magnetometer_xyz_axis();

    int16_t _get_temperature() const {
        // TODO manipulate the bits to convert the 12-bit value to 16-bit
        return _temperature_axis;
    }

    int16_t _get_magnetic_x_axis() const {
        return _magnetometer_x_axis;
    }

    int16_t _get_magnetic_y_axis() const {
        return _magnetometer_y_axis;
    }

    int16_t _get_magnetic_z_axis() const {
        return _magnetometer_z_axis;
    }

public:

    Lsm303DlhcMagnetometer() = default;

    ~Lsm303DlhcMagnetometer() {
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
            host_callback_function function_pointer
            ) {
        _i2c_bus_number = bus_number;
        _i2c_device_address = device_address;
        _sensor_update_period_ms = update_period_ms;
        _device_name = std::move(device_name);

        _host_callback_function = function_pointer;

        data_capture_thread = std::thread(&Lsm303DlhcMagnetometer::_data_capture_worker, this);

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

        mock_device_thread = std::thread(&Lsm303DlhcMagnetometer::_mock_device_emulation, this);

        // wait a little bit for the thread to get started
        std::this_thread::sleep_for(std::chrono::milliseconds (10));

        std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
        this->mock_device_thread_run_cv.notify_one();
        this->mock_run_device_thread = true;
        device_lock.unlock();

        return 1;
    }

};

#endif //LSM303DLHC_H


