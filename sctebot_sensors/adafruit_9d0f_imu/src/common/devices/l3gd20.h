//
// Created by user on 4/9/21.
//

#ifndef L3GD20_H
#define L3GD20_H

#define ENABLE_MOCK_L3GD20_DEVICE 1

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

class L3gd20 {

    // 1101 0101 0xD5 (Read address)
    // 1101 0100 0xD4 (Write address)

private:

    int _device_endian_msb_index = 1;
    int _device_endian_lsb_index = 0;

    int _l3gd20_i2c_bus_number{};
    int _l3gd20_i2c_device_address{};
    int _l3gd20_sensor_update_period_ms{};
    std::string _l3gd20_device_name;

    context_t _l3gd20_i2c_context{};

    uint8_t _l3gd20_control_register_buffer[5] = {0};

    bool run_l3gd20_data_capture_thread = false;
    std::condition_variable l3gd20_data_capture_thread_run_cv;
    std::mutex l3gd20_data_capture_thread_run_mutex;
    std::thread l3gd20_data_capture_thread;

    typedef void (*l3gd20_host_callback_function)(float, float, float);
    l3gd20_host_callback_function _l3gd20_host_callback_function{};

    bool mock_run_l3gd20_device_thread = false;
    std::condition_variable mock_l3gd20_device_thread_run_cv;
    std::mutex mock_l3gd20_device_thread_run_mutex;
    std::thread mock_l3gd20_device_thread;

    int _connect_to_l3gd20() {

        /*
         * setup the i2c context and connect
         */
        _l3gd20_i2c_context = {0};
        if(!i2c_dev_open(&_l3gd20_i2c_context, _l3gd20_i2c_bus_number, _l3gd20_i2c_device_address)) {
            std::cout << "failed to open device\n";
            return 0;
        }

        if(!i2c_is_connected(&_l3gd20_i2c_context)) {
            std::cout << "failed to connect to device\n";
            return 0;
        }

#if ENABLE_MOCK_L3GD20_DEVICE
        mock_load_l3gd20_data();
#endif

        // try read whoami
        uint8_t chip_id[1] = {0};
        buffer_t inbound_message = {
                .bytes = chip_id,
                .size = sizeof(chip_id)
        };

        uint8_t register_address;
        register_address = L3gd20::Addresses::Registers::WHO_AM_I;
        i2c_recv(&_l3gd20_i2c_context, &inbound_message, register_address);

        if(chip_id[0] != L3gd20::MagicNumbers::WhoAmI::WHO_AM_I) {
            std::cout << "failed to read device who am I register\n";
            return 0;
        }
        return 1;
    }

    int _init_l3gd20();

    int _close_device() {

        i2c_dev_close(&_l3gd20_i2c_context, _l3gd20_i2c_bus_number);

        return 0;
    }

    void _l3gd20_data_capture_worker();

    void _mock_l3gd20_device_emulation();

    int _measurement_completed_ok();

public:

    L3gd20() = default;

    ~L3gd20() {

        this->_close_device();

        this->run_l3gd20_data_capture_thread = false;

        std::unique_lock<std::mutex> data_lock(this->l3gd20_data_capture_thread_run_mutex);
        this->l3gd20_data_capture_thread_run_cv.notify_one();
        data_lock.unlock();

        if(l3gd20_data_capture_thread.joinable()) {
            l3gd20_data_capture_thread.join();
        }

        this->mock_run_l3gd20_device_thread = false;

        std::unique_lock<std::mutex> device_lock(this->mock_l3gd20_device_thread_run_mutex);
        this->mock_l3gd20_device_thread_run_cv.notify_one();
        device_lock.unlock();

        if(mock_l3gd20_device_thread.joinable()) {
            mock_l3gd20_device_thread.join();
        }

    }

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
            INT1_DURATION = 0x38
        } Registers;
    };

    class MagicNumbers {
    public:
        typedef enum WhoAmI_t {
            WHO_AM_I = 0xD7 //0xD4
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
            POWER_DOWN_ENABLE = (1 << 3),
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
    };

    int config_l3gd20(int bus_number, int device_address, int update_period_ms, std::string device_name, l3gd20_host_callback_function function_pointer) {
        _l3gd20_i2c_bus_number = bus_number;
        _l3gd20_i2c_device_address = device_address;
        _l3gd20_sensor_update_period_ms = update_period_ms;
        _l3gd20_device_name = std::move(device_name);

        _l3gd20_host_callback_function = function_pointer;

        l3gd20_data_capture_thread = std::thread(&L3gd20::_l3gd20_data_capture_worker, this);

        return 0;
    }

    int connect_to_device() {
        int status = 1;

        status &= this->_connect_to_l3gd20();

        return status;
    }

    int init_device() {
        this->_init_l3gd20();

        return 1;
    }

    int mock_load_l3gd20_data() {
        /*
         * @77 bmp180 (?)
         * @6b l3gd20 (?)
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

        /* @19
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

        /* @1e
             0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f    0123456789abcdef
        00: 10 20 03 00 2b fe 7d 00 e2 03 48 34 33 00 00 3c    ? ?.+?}.??H43..<
        10: 00 00 00 00 00 00 00 00 00 00 00 83 9c 85 e8 10    ...........?????
        20: 00 00 00 00 00 00 00 00 10 00 00 00 00 00 00 00    ........?.......
        30: 00 00 00 1c 84 65 55 00 a0 00 07 00 00 00 00 00    ...??eU.?.?.....
        40: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        50: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        60: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        70: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        80: 10 20 03 00 2b fe 7d 00 e2 03 48 34 33 00 00 3c    ? ?.+?}.??H43..<
        90: 00 00 00 00 00 00 00 00 00 00 00 83 9c 85 e8 10    ...........?????
        a0: 00 00 00 00 00 00 00 00 10 00 00 00 00 00 00 00    ........?.......
        b0: 00 00 00 1c 84 65 55 00 a0 00 07 00 00 00 00 00    ...??eU.?.?.....
        c0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        d0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        e0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        f0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        */

        /* @6b
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

            //std::cout << "i " << i << std::endl;
            send_buffer[i + (i * 1)] = (_mock_device_memory[i] >> 8) & 0xff;
            send_buffer[i + (i * 1) + 1] = (_mock_device_memory[i]) & 0xff;
        }
        //std::memcpy(&send_buffer, _mock_device_memory, sizeof(_mock_device_memory));

        buffer_t outbound_message = {
                .bytes = send_buffer,
                .size = sizeof(send_buffer)
        };

        int8_t register_address = 0x00;

        if (i2c_send(&_l3gd20_i2c_context, &outbound_message, register_address)) {
            std::cout << "loaded mock data for device OK\n";
            return 0;
        }
        else {
            std::cout << "loaded mock data for device FAILED\n";
            return -1;
        }
    }

    int mock_run_l3gd20_device_emulation() {

        mock_l3gd20_device_thread = std::thread(&L3gd20::_mock_l3gd20_device_emulation, this);

        // wait a little bit for the thread to get started
        std::this_thread::sleep_for(std::chrono::milliseconds (10));

        std::unique_lock<std::mutex> device_lock(this->mock_l3gd20_device_thread_run_mutex);
        this->mock_l3gd20_device_thread_run_cv.notify_one();
        this->mock_run_l3gd20_device_thread = true;
        device_lock.unlock();

        return 1;
    }

};

#endif //L3GD20_H
