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

#include <math.h>

#include <boost/log/core.hpp>
#include <boost/log/trivial.hpp>
#include <boost/log/expressions.hpp>

namespace logging = boost::log;

#include "i2c_linux.h"

class Lsm303DlhcAccelerometer {

public:

    typedef enum OutputDataRates_t {
        ODR_1P0HZ = 0,
        ODR_10P0HZ,
        ODR_25P0HZ,
        ODR_50P0HZ,
        ODR_100P0HZ,
        ODR_200P0HZ,
        ODR_400P0HZ,
        ODR_1620P0HZ,
        ODR_1344P0HZ
    } OutputDataRates;

    typedef enum HighPassFilterCutoff_t {
        MIN_CUT_OFF = 0,
        MED_CUT_OFF,
        HIGH_CUT_OFF,
        MAX_CUT_OFF
    } HighPassFilterCutoff;

    typedef enum SensorAccelerationFullScale_t {
        PN_2G = 0,
        PN_4G,
        PN_8G,
        PN_16G
    } SensorAccelerationFullScale;

    std::map<int, int> data_rate_sample_rate{
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_1P0HZ,    1},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_10P0HZ,   10},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_25P0HZ,   25},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_50P0HZ,   50},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_100P0HZ,  100},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_200P0HZ,  200},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_400P0HZ,  400},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_1620P0HZ, 1620},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_1344P0HZ, 1344},
    };

private:

    std::map<int, int> sample_rate_to_register_bitmask{
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_1P0HZ,    Lsm303DlhcAccelerometer::BitMasks::DataRates::ODR_1P0HZ_BM},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_10P0HZ,   Lsm303DlhcAccelerometer::BitMasks::DataRates::ODR_10P0HZ_BM},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_25P0HZ,   Lsm303DlhcAccelerometer::BitMasks::DataRates::ODR_25P0HZ_BM},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_50P0HZ,   Lsm303DlhcAccelerometer::BitMasks::DataRates::ODR_50P0HZ_BM},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_100P0HZ,  Lsm303DlhcAccelerometer::BitMasks::DataRates::ODR_100P0HZ_BM},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_200P0HZ,  Lsm303DlhcAccelerometer::BitMasks::DataRates::ODR_200P0HZ_BM},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_400P0HZ,  Lsm303DlhcAccelerometer::BitMasks::DataRates::ODR_400P0HZ_BM},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_1620P0HZ, Lsm303DlhcAccelerometer::BitMasks::DataRates::ODR_1620P0HZ_BM},
            {Lsm303DlhcAccelerometer::OutputDataRates::ODR_1344P0HZ, Lsm303DlhcAccelerometer::BitMasks::DataRates::ODR_1344P0HZ_BM}
    };

    std::map<int, int> high_pass_filter_cutoff_register_bitmasks{
            {Lsm303DlhcAccelerometer::HighPassFilterCutoff::MIN_CUT_OFF, Lsm303DlhcAccelerometer::BitMasks::HighPassFilterCutoff::MIN_CUT_OFF_BM},
            {Lsm303DlhcAccelerometer::HighPassFilterCutoff::MED_CUT_OFF, Lsm303DlhcAccelerometer::BitMasks::HighPassFilterCutoff::MED_CUT_OFF_BM},
            {Lsm303DlhcAccelerometer::HighPassFilterCutoff::HIGH_CUT_OFF, Lsm303DlhcAccelerometer::BitMasks::HighPassFilterCutoff::HIGH_CUT_OFF_BM},
            {Lsm303DlhcAccelerometer::HighPassFilterCutoff::MAX_CUT_OFF, Lsm303DlhcAccelerometer::BitMasks::HighPassFilterCutoff::MAX_CUT_OFF_BM}
    };

    std::map<int, int> full_scale_acceleration_range_register_bitmasks{
            {Lsm303DlhcAccelerometer::SensorAccelerationFullScale::PN_2G, Lsm303DlhcAccelerometer::BitMasks::SensorAccelerationFullScale::FS_2GPN_SEL_BM},
            {Lsm303DlhcAccelerometer::SensorAccelerationFullScale::PN_4G, Lsm303DlhcAccelerometer::BitMasks::SensorAccelerationFullScale::FS_4GPN_SEL_BM},
            {Lsm303DlhcAccelerometer::SensorAccelerationFullScale::PN_8G, Lsm303DlhcAccelerometer::BitMasks::SensorAccelerationFullScale::FS_8GPN_SEL_BM},
            {Lsm303DlhcAccelerometer::SensorAccelerationFullScale::PN_16G, Lsm303DlhcAccelerometer::BitMasks::SensorAccelerationFullScale::FS_16GPN_SEL_BM}
    };

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

        typedef enum DataRates_t {
            ODR_1P0HZ_BM = (1 << 6),
            ODR_10P0HZ_BM = (2 << 6),
            ODR_25P0HZ_BM = (3 << 6),
            ODR_50P0HZ_BM = (4 << 6),
            ODR_100P0HZ_BM = (5 << 6),
            ODR_200P0HZ_BM = (6 << 6),
            ODR_400P0HZ_BM = (7 << 6),
            ODR_1620P0HZ_BM = (8 << 6),
            ODR_1344P0HZ_BM = (9 << 6)
        } DataRates;

        typedef enum HighPassFilterCutoff_t {
            MIN_CUT_OFF_BM = (0 << 4),
            MED_CUT_OFF_BM = (1 << 4),
            HIGH_CUT_OFF_BM = (2 << 4),
            MAX_CUT_OFF_BM = (3 << 4)
        } HighPassFilterCutoff;

        typedef enum SensorAccelerationFullScale_t {
            FS_2GPN_SEL_BM = (0 << 4),
            FS_4GPN_SEL_BM = (1 << 4),
            FS_8GPN_SEL_BM = (2 << 4),
            FS_16GPN_SEL_BM = (3 << 4)
        } SensorAccelerationFullScale;

        typedef enum ControlRegister1_t {
            ODR_POWER_DOWN = 0b00000000,
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

    std::mutex accelerometer_data_mutex;
    uint8_t _status_register{0};

    int16_t _accelerometer_x_axis_bytes{0};
    int16_t _accelerometer_y_axis_bytes{0};
    int16_t _accelerometer_z_axis_bytes{0};
    float accelerometer_x_axis_g{0};
    float accelerometer_y_axis_g{0};
    float accelerometer_z_axis_g{0};

    float _linear_acceleration_sensitivity{0};

    bool run_data_capture_thread = false;
    std::condition_variable data_capture_thread_run_cv;
    std::mutex data_capture_thread_run_mutex;
    std::thread data_capture_thread;

    typedef void (*host_callback_function)(
            float x_accel_axis, float y_accel_axis, float z_accel_axis
            );

    host_callback_function _host_callback_function{};

    bool mock_run_device_thread = false;
    std::condition_variable mock_device_thread_run_cv;
    std::mutex mock_device_thread_run_mutex;
    std::thread mock_device_thread;

    int _init_device(
            Lsm303DlhcAccelerometer::OutputDataRates_t,
            Lsm303DlhcAccelerometer::HighPassFilterCutoff_t,
            Lsm303DlhcAccelerometer::SensorAccelerationFullScale_t
            );

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

    void _update_accelerometer_xyz_axis();

    uint8_t _update_accelerometer_status();

    int16_t _get_accel_x_axis() const {
        return _accelerometer_x_axis_bytes;
    }

    int16_t _get_accel_y_axis() const {
        return _accelerometer_y_axis_bytes;
    }

    int16_t _get_accel_z_axis() const {
        return _accelerometer_z_axis_bytes;
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
            std::string device_name,
            host_callback_function function_pointer
            ) {
        _i2c_bus_number = bus_number;
        _i2c_device_address = device_address;
        _sensor_update_period_ms = 0;
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

    int init_device(
            Lsm303DlhcAccelerometer::OutputDataRates_t output_data_rate,
            Lsm303DlhcAccelerometer::HighPassFilterCutoff_t high_pass_filter_cutoff,
            Lsm303DlhcAccelerometer::SensorAccelerationFullScale_t sensor_full_scale_accelerometer_range
            ) {

        int data_rate = this->data_rate_sample_rate[output_data_rate];

        float rate = (1.0f / float(data_rate)) * (1.0f / 3);

        this->_sensor_update_period_ms = int(rate * 1000);

        this->_init_device(output_data_rate, high_pass_filter_cutoff, sensor_full_scale_accelerometer_range);

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

public:

    typedef enum OutputDataRates_t {
        ODR_0P75_HZ = 0,
        ODR_1P5_HZ,
        ODR_3P0_HZ,
        ODR_7P5_HZ,
        ODR_15P0_HZ,
        ODR_30P0_HZ,
        ODR_75P0_HZ,
        ODR_220P0_HZ
    } OutputDataRates;

    typedef enum SensorMagnetometerFullScale_t {
        PN_1P3G = 0,
        PN_1P9G,
        PN_2P5G,
        PN_4P0G,
        PN_4P7G,
        PN_5P6G,
        PN_8P1G
    } SensorMagnetometerFullScale;

    std::map<int, float> data_rate_sample_rate{
            {Lsm303DlhcMagnetometer::ODR_0P75_HZ, 0.75},
            {Lsm303DlhcMagnetometer::ODR_1P5_HZ, 1.5},
            {Lsm303DlhcMagnetometer::ODR_3P0_HZ, 3.0},
            {Lsm303DlhcMagnetometer::ODR_7P5_HZ, 7.5},
            {Lsm303DlhcMagnetometer::ODR_15P0_HZ, 15.0},
            {Lsm303DlhcMagnetometer::ODR_30P0_HZ, 30.0},
            {Lsm303DlhcMagnetometer::ODR_75P0_HZ, 75.0},
            {Lsm303DlhcMagnetometer::ODR_220P0_HZ, 220.0}
    };

private:

    std::map<int, int> sample_rate_to_register_bitmask{
            {Lsm303DlhcMagnetometer::OutputDataRates::ODR_0P75_HZ, Lsm303DlhcMagnetometer::BitMasks::DataRates::DATA_OUTPUT_RATE_0P75_HZ},
            {Lsm303DlhcMagnetometer::OutputDataRates::ODR_1P5_HZ, Lsm303DlhcMagnetometer::BitMasks::DataRates::DATA_OUTPUT_RATE_1P5_HZ},
            {Lsm303DlhcMagnetometer::OutputDataRates::ODR_3P0_HZ, Lsm303DlhcMagnetometer::BitMasks::DataRates::DATA_OUTPUT_RATE_3P0_HZ},
            {Lsm303DlhcMagnetometer::OutputDataRates::ODR_7P5_HZ, Lsm303DlhcMagnetometer::BitMasks::DataRates::DATA_OUTPUT_RATE_7P5_HZ},
            {Lsm303DlhcMagnetometer::OutputDataRates::ODR_15P0_HZ, Lsm303DlhcMagnetometer::BitMasks::DataRates::DATA_OUTPUT_RATE_15P0_HZ},
            {Lsm303DlhcMagnetometer::OutputDataRates::ODR_30P0_HZ, Lsm303DlhcMagnetometer::BitMasks::DataRates::DATA_OUTPUT_RATE_30P0_HZ},
            {Lsm303DlhcMagnetometer::OutputDataRates::ODR_75P0_HZ, Lsm303DlhcMagnetometer::BitMasks::DataRates::DATA_OUTPUT_RATE_75P0_HZ},
            {Lsm303DlhcMagnetometer::OutputDataRates::ODR_220P0_HZ, Lsm303DlhcMagnetometer::BitMasks::DataRates::DATA_OUTPUT_RATE_220P0_HZ}
    };

    std::map<int, int> full_scale_magnetometer_range_register_bitmasks{
            {Lsm303DlhcMagnetometer::SensorMagnetometerFullScale::PN_1P3G, Lsm303DlhcMagnetometer::BitMasks::SensorMagnetometerFullScale::PN_1P3G_BM},
            {Lsm303DlhcMagnetometer::SensorMagnetometerFullScale::PN_1P9G, Lsm303DlhcMagnetometer::BitMasks::SensorMagnetometerFullScale::PN_1P9G_BM},
            {Lsm303DlhcMagnetometer::SensorMagnetometerFullScale::PN_2P5G, Lsm303DlhcMagnetometer::BitMasks::SensorMagnetometerFullScale::PN_2P5G_BM},
            {Lsm303DlhcMagnetometer::SensorMagnetometerFullScale::PN_4P0G, Lsm303DlhcMagnetometer::BitMasks::SensorMagnetometerFullScale::PN_4P0G_BM},
            {Lsm303DlhcMagnetometer::SensorMagnetometerFullScale::PN_4P7G, Lsm303DlhcMagnetometer::BitMasks::SensorMagnetometerFullScale::PN_4P7G_BM},
            {Lsm303DlhcMagnetometer::SensorMagnetometerFullScale::PN_5P6G, Lsm303DlhcMagnetometer::BitMasks::SensorMagnetometerFullScale::PN_5P6G_BM},
            {Lsm303DlhcMagnetometer::SensorMagnetometerFullScale::PN_8P1G, Lsm303DlhcMagnetometer::BitMasks::SensorMagnetometerFullScale::PN_8P1G_BM}
    };

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

        typedef enum DataRates_t {
            DATA_OUTPUT_RATE_0P75_HZ = (0 << 2),
            DATA_OUTPUT_RATE_1P5_HZ = (1 << 2),
            DATA_OUTPUT_RATE_3P0_HZ = (2 << 2),
            DATA_OUTPUT_RATE_7P5_HZ = (3 << 2),
            DATA_OUTPUT_RATE_15P0_HZ = (4 << 2),
            DATA_OUTPUT_RATE_30P0_HZ = (5 << 2),
            DATA_OUTPUT_RATE_75P0_HZ = (6 << 2),
            DATA_OUTPUT_RATE_220P0_HZ = (7 << 2)
        } DataRates;

        typedef enum SensorMagnetometerFullScale_t {
            PN_1P3G_BM = (1 << 5),
            PN_1P9G_BM = (2 << 5),
            PN_2P5G_BM = (3 << 5),
            PN_4P0G_BM = (4 << 5),
            PN_4P7G_BM = (5 << 5),
            PN_5P6G_BM = (6 << 5),
            PN_8P1G_BM = (7 << 5)
        } SensorMagnetometerFullScale;

        typedef enum CrARegM_t {
            TEMP_EN = (1 << 7)
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

    std::mutex magnetometer_data_mutex;
    uint8_t _status_register{0};

    uint8_t _cra_reg_m[1] = {0};
    uint8_t _crb_reg_m[1] = {0};
    uint8_t _mr_reg_m[1] = {0};

    int16_t _magnetometer_x_axis_bytes{0};
    int16_t _magnetometer_y_axis_bytes{0};
    int16_t _magnetometer_z_axis_bytes{0};
    float magnetometer_x_axis_gauss{0};
    float magnetometer_y_axis_gauss{0};
    float magnetometer_z_axis_gauss{0};

    int16_t _temperature_axis_bytes{0};
    float temperature_axis_degrees_c{0};

    float _mag_xy_gain_config{0};
    float _mag_z_gain_config{0};

    bool run_data_capture_thread = false;
    std::condition_variable data_capture_thread_run_cv;
    std::mutex data_capture_thread_run_mutex;
    std::thread data_capture_thread;

    typedef void (*host_callback_function)(
            float temperature_deg_c,
            float x_mag_axis, float y_mag_axis, float z_mag_axis
            );

    host_callback_function _host_callback_function{};

    bool mock_run_device_thread = false;
    std::condition_variable mock_device_thread_run_cv;
    std::mutex mock_device_thread_run_mutex;
    std::thread mock_device_thread;

    int _init_device(
            Lsm303DlhcMagnetometer::OutputDataRates_t,
            Lsm303DlhcMagnetometer::SensorMagnetometerFullScale_t
            );

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

    void _update_temperature_axis();

    void _update_magnetometer_xyz_axis();

    uint8_t _update_magnetometer_status();

    int16_t _get_temperature() const {
        return _temperature_axis_bytes;
    }

    int16_t _get_magnetic_x_axis() const {
        return _magnetometer_x_axis_bytes;
    }

    int16_t _get_magnetic_y_axis() const {
        return _magnetometer_y_axis_bytes;
    }

    int16_t _get_magnetic_z_axis() const {
        return _magnetometer_z_axis_bytes;
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
            //int update_period_ms,
            std::string device_name,
            host_callback_function function_pointer
            ) {
        _i2c_bus_number = bus_number;
        _i2c_device_address = device_address;
        _sensor_update_period_ms = 0;
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

    int init_device(
            Lsm303DlhcMagnetometer::OutputDataRates_t output_data_rates,
            Lsm303DlhcMagnetometer::SensorMagnetometerFullScale_t magnetometer_full_scale
            ) {

        float data_rate = this->data_rate_sample_rate[output_data_rates];

        float rate = (1.0f / float(data_rate)) * (1.0f / 3);

        this->_sensor_update_period_ms = int(rate * 1000);

        this->_init_device(output_data_rates, magnetometer_full_scale);

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


