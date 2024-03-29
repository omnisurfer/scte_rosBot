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
#include <cmath>

#include "i2c_linux/i2c_linux.h"
#include "utils/boost_logging.h"
#include "utils/register_utils.h"

#define DEG_TO_RAD (M_PI/180.0)

class L3gd20Gyro {

    // 1101 0101 0xD5 (Read address)
    // 1101 0100 0xD4 (Write address)

public:

    //TODO Correct variance calculation TBD
    /* from datasheet */
    constexpr static const double rate_noise_density = 0.011 * DEG_TO_RAD;

    /* probably the wrong values for this matrix */
    constexpr static const double angular_field_covariance[9] = {
            rate_noise_density, 0.0, 0.0,
            0.0, rate_noise_density, 0.0,
            0.0, 0.0, rate_noise_density
    };

    typedef enum OutputDataRates_t {
        ODR_12P5HZ = 0,
        ODR_25P0HZ,
        ODR_50P0HZ,
        ODR_100P0HZ,
        ODR_200P0HZ,
        ODR_400P0HZ,
        ODR_800P0HZ
    } OutputDataRates;

    typedef enum BandwidthCutOff_t {
        MIN_CUT_OFF = 0,
        MED_CUT_OFF,
        HIGH_CUT_OFF,
        MAX_CUT_OFF
    } BandwidthCutOff;

    std::map<int, float> data_rate_sample_rate{
            {L3gd20Gyro::OutputDataRates::ODR_12P5HZ, 12.5},
            {L3gd20Gyro::OutputDataRates::ODR_25P0HZ, 25.0},
            {L3gd20Gyro::OutputDataRates::ODR_50P0HZ, 50.0},
            {L3gd20Gyro::OutputDataRates::ODR_100P0HZ, 100.0},
            {L3gd20Gyro::OutputDataRates::ODR_200P0HZ, 200.0},
            {L3gd20Gyro::OutputDataRates::ODR_400P0HZ, 400.0},
            {L3gd20Gyro::OutputDataRates::ODR_800P0HZ, 800.0},
    };

private:

    std::map<int, int> sample_rate_to_register_bitmasks{
            {L3gd20Gyro::OutputDataRates::ODR_12P5HZ, L3gd20Gyro::BitMasks::DataRates::ODR_12P5HZ_BM},
            {L3gd20Gyro::OutputDataRates::ODR_25P0HZ, L3gd20Gyro::BitMasks::DataRates::ODR_25P0HZ_BM},
            {L3gd20Gyro::OutputDataRates::ODR_50P0HZ, L3gd20Gyro::BitMasks::DataRates::ODR_50P0HZ_BM},
            {L3gd20Gyro::OutputDataRates::ODR_100P0HZ, L3gd20Gyro::BitMasks::DataRates::ODR_100P0HZ_BM},
            {L3gd20Gyro::OutputDataRates::ODR_200P0HZ, L3gd20Gyro::BitMasks::DataRates::ODR_200P0HZ_BM},
            {L3gd20Gyro::OutputDataRates::ODR_400P0HZ, L3gd20Gyro::BitMasks::DataRates::ODR_400P0HZ_BM},
            {L3gd20Gyro::OutputDataRates::ODR_800P0HZ, L3gd20Gyro::BitMasks::DataRates::ODR_800P0HZ_BM}
    };

    std::map<int, int> bandwidth_cut_off_to_register_bitmasks{
            {L3gd20Gyro::BandwidthCutOff::MIN_CUT_OFF, L3gd20Gyro::BitMasks::BandwidthCutOff::MIN_CUT_OFF_BM},
            {L3gd20Gyro::BandwidthCutOff::MED_CUT_OFF, L3gd20Gyro::BitMasks::BandwidthCutOff::MED_CUT_OFF_BM},
            {L3gd20Gyro::BandwidthCutOff::HIGH_CUT_OFF, L3gd20Gyro::BitMasks::BandwidthCutOff::HIGH_CUT_OFF_BM},
            {L3gd20Gyro::BandwidthCutOff::MAX_CUT_OFF, L3gd20Gyro::BitMasks::BandwidthCutOff::MAX_CUT_OFF_BM}
    };

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

        typedef enum DataRates_t {
            ODR_12P5HZ_BM = (0 << 6),
            ODR_25P0HZ_BM = (1 << 6),
            ODR_50P0HZ_BM = (2 << 6),
            ODR_100P0HZ_BM = (0 << 6),
            ODR_200P0HZ_BM = (1 << 6),
            ODR_400P0HZ_BM = (2 << 6),
            ODR_800P0HZ_BM = (3 << 6)
        } DataRates;

        typedef enum BandwidthCutOff_t {
            MIN_CUT_OFF_BM = (0 << 4),
            MED_CUT_OFF_BM = (1 << 4),
            HIGH_CUT_OFF_BM = (2 << 4),
            MAX_CUT_OFF_BM = (3 << 4)
        } BandwidthCutOff;

        typedef enum ControlRegister1_t {
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

        typedef enum LowODRRegister_t {
            LOW_ODR = (1 << 0),
            SW_RES = (1 << 2)
        } LowODRRegister;

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

    std::mutex _i2c_device_mutex;
    int _i2c_bus_number{};
    int _i2c_device_address{};
    int _sensor_update_period_ms{};
    std::string _device_name;

    context_t _i2c_device_context{};

    bool _enable_load_mock_data = false;

    std::mutex gyroscope_data_mutex;
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
    bool is_running_data_capture_thread = false;
    std::condition_variable data_capture_thread_run_cv;
    std::mutex data_capture_thread_run_mutex;
    std::thread data_capture_thread;

    typedef void (*host_callback_function)(
            const float temperature,
            const float x_axis, const float y_axis, const float z_axis
            );
    host_callback_function _host_callback_function{};

    bool mock_run_device_thread = false;
    bool is_running_mock_device_thread = false;
    std::condition_variable mock_device_thread_run_cv;
    std::mutex mock_device_thread_run_mutex;
    std::thread mock_device_thread;

    std::mutex& _data_capture_worker_execute_cycle_mutex;
    std::condition_variable& _data_capture_worker_execute_cycle_conditional_variable;

    int _init_device(L3gd20Gyro::OutputDataRates_t output_data_rate, L3gd20Gyro::BandwidthCutOff_t bandwidth_cutoff);

    int _connect_to_device() {
        std::string device_name = this->_device_name;

        /*
         * setup the i2c context and connect
         */
        _i2c_device_context = {0};
        if(!open_i2c_dev(_i2c_bus_number, _i2c_device_address)) {
            BOOST_LOG_TRIVIAL(error) << device_name << ": failed to open";
            return 0;
        }

        if(!is_i2c_dev_connected()) {
            BOOST_LOG_TRIVIAL(error) << device_name << ": failed to connect";
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
        receive_i2c(&inbound_message, register_address);

        if(chip_id[0] != L3gd20Gyro::MagicNumbers::WhoAmI::WHO_AM_I) {
            BOOST_LOG_TRIVIAL(error) << device_name << ": failed to read device WHO_AM_I register";
            return 0;
        }
        return 1;
    }

    int _close_device() {

        close_i2c_dev(_i2c_bus_number);

        return 0;
    }

    int receive_i2c(buffer_t *data, uint8_t register_address);
    int send_i2c(buffer_t *data, uint8_t register_address);
    int open_i2c_dev(int device_number, int slave_address);
    int is_i2c_dev_connected();
    void close_i2c_dev(int bus_number);

    int _mock_load_data() {
        std::string device_name = this->_device_name;
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
                0x0700, 0x0000, 0x0000, 0x040F, 0x18ff, 0xed00, 0xf3fe, 0x0020,
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

        if (send_i2c(&outbound_message, register_address)) {
            BOOST_LOG_TRIVIAL(debug) << device_name << ": loaded mock data for device OK";
            return 0;
        }
        else {
            BOOST_LOG_TRIVIAL(debug) <<  device_name << ": loaded mock data for device FAILED";
            return -1;
        }
    }

    void _data_capture_worker();

    void _mock_device_emulation_worker();

    void _update_temperature_axis();

    void _update_angular_rate_xyz_axis();

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

    uint8_t _update_gyroscope_status();

public:

    L3gd20Gyro(std::mutex& worker_execute_mutex, std::condition_variable& worker_execute_conditional_variable):
            _data_capture_worker_execute_cycle_mutex(worker_execute_mutex),
            _data_capture_worker_execute_cycle_conditional_variable(worker_execute_conditional_variable){
    }

    ~L3gd20Gyro() {

        BOOST_LOG_TRIVIAL(debug) << this->_device_name << ": destructor running";

        this->_shutdown_device();

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

        data_capture_thread = std::thread(&L3gd20Gyro::_data_capture_worker, this);

        return 0;
    }

    int connect_to_device() {
        int status = 1;

        status &= this->_connect_to_device();

        return status;
    }

    int init_device(L3gd20Gyro::OutputDataRates_t output_data_rate, L3gd20Gyro::BandwidthCutOff_t bandwidth_cutoff) {

        float data_rate_hz = this->data_rate_sample_rate[output_data_rate];

        float rate_s = (1.0f / float(data_rate_hz));

        this->_sensor_update_period_ms = int(rate_s * 1000);

        this->_init_device(output_data_rate, bandwidth_cutoff);

        return 1;
    }

    void _shutdown_device() {

        std::unique_lock<std::mutex> execute_cycle_lock(this->_data_capture_worker_execute_cycle_mutex);
        this->_data_capture_worker_execute_cycle_conditional_variable.notify_all();
        execute_cycle_lock.unlock();

        bool data_capture_thread_was_running = false;
        std::unique_lock<std::mutex> data_lock(this->data_capture_thread_run_mutex);
        {
            if(is_running_data_capture_thread) {
                data_capture_thread_was_running = is_running_data_capture_thread;
                this->run_data_capture_thread = false;
                this->data_capture_thread_run_cv.notify_one();
            }
            data_lock.unlock();
        }

        if(data_capture_thread_was_running) {
            if(data_capture_thread.joinable()) {
                data_capture_thread.join();

                BOOST_LOG_TRIVIAL(debug) << this->_device_name << ": data_capture_thread joined";
            }
        }

        bool mock_device_thread_was_running = false;
        std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
        {
            if(is_running_mock_device_thread) {
                mock_device_thread_was_running = is_running_mock_device_thread;
                this->mock_run_device_thread = false;
                this->mock_device_thread_run_cv.notify_one();
            }
            device_lock.unlock();
        }

        if(mock_device_thread_was_running) {
            if (mock_device_thread.joinable()) {
                mock_device_thread.join();

                BOOST_LOG_TRIVIAL(debug) << this->_device_name << ": mock_device_thread joined";
            }
        }

        this->_close_device();
    }

    void enable_load_mock_data();

    int mock_run_device_emulation() {

        mock_device_thread = std::thread(&L3gd20Gyro::_mock_device_emulation_worker, this);

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
