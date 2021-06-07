//
// Created by user on 4/9/21.
//

#ifndef BMP180_H
#define BMP180_H

#define ENABLE_MOCK_BMP180_DEVICE 1

// some code taken from my github:
// https://github.com/omnisurfer/IMUController_firmware_AtmelC/blob/master/IMU/src/app_components/sensors/bmp180/bmp180.c

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

class Bmp180 {

private:

    int _device_endian_msb_index = 1;
    int _device_endian_lsb_index = 0;

    int _bmp180_i2c_bus_number{};
    int _bmp180_i2c_device_address{};
    int _bmp180_sensor_update_period_ms{};
    std::string _bmp180_device_name;

    context_t _bmp180_i2c_context{};

    uint8_t _bmp180_calibration_data_buffer[11 * 2] = {0};
    uint16_t _bmp180_long_uncompensated_temperature{};
    uint16_t _bmp180_long_uncompensated_pressure{};
    uint8_t  _bmp180_short_uncompensated_pressure_xlsb{};

    bool run_bmp180_data_capture_thread = false;
    std::condition_variable bmp180_data_capture_thread_run_cv;
    std::mutex bmp180_data_capture_thread_run_mutex;
    std::thread bmp180_data_capture_thread;

    bool mock_run_bmp180_device_thread = false;
    std::condition_variable mock_bmp180_device_thread_run_cv;
    std::mutex mock_bmp180_device_thread_run_mutex;
    std::thread mock_bmp180_device_thread;

    typedef void (*bmp180_host_callback_function)(float, float);
    bmp180_host_callback_function _bmp180_host_callback_function{};

    bool sensor_calibration_read = false;
    //bool sensor_configured = false;

    struct Bmp180CalibrationCoefficients_t {
        union {
            int16_t AC1;
            uint8_t AC1_ba[2];
        };

        union {
            int16_t AC2;
            uint8_t AC2_ba[2];
        };

        union {
            int16_t AC3;
            uint8_t AC3_ba[2];
        };

        union {
            uint16_t AC4;
            uint8_t AC4_ba[2];
        };

        union {
            uint16_t AC5;
            uint8_t AC5_ba[2];
        };

        union {
            uint16_t AC6;
            uint8_t AC6_ba[2];
        };

        union {
            int16_t B1;
            uint8_t B1_ba[2];
        };

        union {
            int16_t B2;
            uint8_t B2_ba[2];
        };

        union {
            int16_t MB;
            uint8_t MB_ba[2];
        };

        union {
            int16_t MC;
            uint8_t MC_ba[2];
        };

        union {
            int16_t MD;
            uint8_t MD_ba[2];
        };

    } Bmp180CalibrationCoefficients;

    struct Bmp180SharedCoefficients_t {
        union {
            float B5;
        };
    } Bmp180SharedCoefficients;

    int _connect_to_bmp180() {

        /*
         * setup the i2c context and connect
         */
        _bmp180_i2c_context = {0};
        if(!i2c_dev_open(&_bmp180_i2c_context, _bmp180_i2c_bus_number, _bmp180_i2c_device_address)) {
            std::cout << "failed to open device\n";
            return 0;
        }

        if(!i2c_is_connected(&_bmp180_i2c_context)) {
            std::cout << "failed to connect to device\n";
            return 0;
        }

        mock_load_bmp180_calibration_data();

        // try read chip id
        uint8_t chip_id[1] = {0};
        buffer_t inbound_message = {
                .bytes = chip_id,
                .size = sizeof(chip_id)
        };

        uint8_t register_address;
        register_address = Bmp180::Addresses::DataRegisters::CHIP_ID;
        i2c_recv(&_bmp180_i2c_context, &inbound_message, register_address);

        if(chip_id[0] != Bmp180::Commands::ChipId::CHIP_ID) {
            std::cout << "failed to read device chip id\n";
            return 0;
        }

        return 1;
    }

    int _init_bmp180();

    int _close_device() {

        i2c_dev_close(&_bmp180_i2c_context, _bmp180_i2c_bus_number);

        return 0;
    }

    void _bmp180_data_capture_worker();

    void _mock_bmp180_device_emulation();

    void _get_bmp180_temperature();

    void _get_bmp180_pressure();

    int _measurement_completed_ok();

    uint16_t _get_bmp180_uncompensated_temperature_count() const {
        return _bmp180_long_uncompensated_temperature;
    }

    uint16_t _get_bmp180_uncompensated_pressure() const {
        return _bmp180_long_uncompensated_pressure;
    }

    uint8_t _get_bmp180_uncompensated_pressure_xlsb() const {
        return _bmp180_short_uncompensated_pressure_xlsb;
    }

    uint8_t* _get_bmp180_calibration_buffer_address() {
        return _bmp180_calibration_data_buffer;
    }

    int _get_bmp180_calibration_buffer_size() {
        return sizeof(_bmp180_calibration_data_buffer);
    }

    // Note, temperature must be read first, then pressure so that B5 value can be populated
    int _calculate_temperature(uint16_t uncompensated_temperature, float &temperature) {

        if(!sensor_calibration_read) {
            std::cout << "sensor calibration not read yet\n";
            return 0;
        }

        // refer to formula found on page 15 of datasheet
        float _temperature = uncompensated_temperature;

        float X1 = (_temperature - (float)Bmp180CalibrationCoefficients.AC6) * ((float)Bmp180CalibrationCoefficients.AC5/32768);
        float X2 = ((float)Bmp180CalibrationCoefficients.MC * 2048) / (X1 + (float)Bmp180CalibrationCoefficients.MD);

        Bmp180SharedCoefficients.B5 = X1 + X2;

        // multiplying by 0.1 here to convert to degrees C instead of degrees 0.1C
        _temperature = static_cast<float>(((Bmp180SharedCoefficients.B5 + 8) / 16)) * 0.1;

        temperature = _temperature;

        //std::cout << "calc temp (C): " << temperature << std::endl;

        return 1;

    }

    int _calculate_pressure(uint16_t uncompensated_pressure, uint8_t uncompensated_pressure_xlsb, float &pressure) const {

        if(!sensor_calibration_read) {
            std::cout << "sensor calibration not read yet\n";
            return 0;
        }

        // uint8_t ut_l = 0x23;
        // uint8_t ut_h = 0x5d;

        // long _pressure = ((ut_h << 16) + (ut_l << 8) + uncompensated_pressure_xlsb) >> (8 - 0);
        long _pressure = (uncompensated_pressure << 8) >> (8 - 0);
        _pressure += uncompensated_pressure_xlsb;

        // refer to formula found on page 15 of datasheet
        // all these operations in powers of 2 seem to be "hiding" bitshifts. wonder if the compiler will notice
        // this...
        float _p = 0.00f,
                B6 = Bmp180SharedCoefficients.B5 - 4000,

                X1 = ((float)Bmp180CalibrationCoefficients.B2 * ((B6 * B6) / 4096)) / 2048,
                X2 = ((float)Bmp180CalibrationCoefficients.AC2 * B6) / 2048,
                X3 = X1 + X2,
                B3 = (((float)Bmp180CalibrationCoefficients.AC1 * 4 + X3) + 2) / 4;

        X1 = ((float)Bmp180CalibrationCoefficients.AC3 * B6) / 8192;
        X2 = ((float)Bmp180CalibrationCoefficients.B1 * (B6 * B6 / 4096)) / 65536;
        X3 = ((X1 + X2) + 2) / 4;

        unsigned long B4 = (Bmp180CalibrationCoefficients.AC4 * (unsigned)(X3 * 32768)) / 32768;

        //note _B7 has a scaling option that is tied to the oversample (OSS). I am assuming 0 oversampling for now
        unsigned long B7 = ((unsigned)_pressure - B3) * 50000;

        if(B7 < 0x80000000) {
            _p = (B7 * 2) / B4;
        }
        else {
            _p = (B7 / B4) * 2;
        }

        X1 = (_p/256) * (_p/256);
        X1 = (X1 * 3038) / 65536;
        X2 = (-7357 * _p) / 65536;

        _p = _p + ((X1 + X2 + 3791) / 16);

        pressure = _p;

        //std::cout << "calc pressure (Pa): " << pressure << std::endl;

        return 1;
    }

    int _init_bmp180_calibration_coefficients(char *bytes, uint8_t length) {
        std::cout << "_init_bmp180_calibration_coefficients\n";

        if(length < 0 or length > 22) {
            return 0;
        }

        // device seems to be big endian
        Bmp180CalibrationCoefficients.AC1_ba[_device_endian_msb_index] = bytes[0];
        Bmp180CalibrationCoefficients.AC1_ba[_device_endian_lsb_index] = bytes[1];

        Bmp180CalibrationCoefficients.AC2_ba[_device_endian_msb_index] = bytes[2];
        Bmp180CalibrationCoefficients.AC2_ba[_device_endian_lsb_index] = bytes[3];

        Bmp180CalibrationCoefficients.AC3_ba[_device_endian_msb_index] = bytes[4];
        Bmp180CalibrationCoefficients.AC3_ba[_device_endian_lsb_index] = bytes[5];

        Bmp180CalibrationCoefficients.AC4_ba[_device_endian_msb_index] = bytes[6];
        Bmp180CalibrationCoefficients.AC4_ba[_device_endian_lsb_index] = bytes[7];

        Bmp180CalibrationCoefficients.AC5_ba[_device_endian_msb_index] = bytes[8];
        Bmp180CalibrationCoefficients.AC5_ba[_device_endian_lsb_index] = bytes[9];

        Bmp180CalibrationCoefficients.AC6_ba[_device_endian_msb_index] = bytes[10];
        Bmp180CalibrationCoefficients.AC6_ba[_device_endian_lsb_index] = bytes[11];

        Bmp180CalibrationCoefficients.B1_ba[_device_endian_msb_index] = bytes[12];
        Bmp180CalibrationCoefficients.B1_ba[_device_endian_lsb_index] = bytes[13];

        Bmp180CalibrationCoefficients.B2_ba[_device_endian_msb_index] = bytes[14];
        Bmp180CalibrationCoefficients.B2_ba[_device_endian_lsb_index] = bytes[15];

        Bmp180CalibrationCoefficients.MB_ba[_device_endian_msb_index] = bytes[16];
        Bmp180CalibrationCoefficients.MB_ba[_device_endian_lsb_index] = bytes[17];

        Bmp180CalibrationCoefficients.MC_ba[_device_endian_msb_index] = bytes[18];
        Bmp180CalibrationCoefficients.MC_ba[_device_endian_lsb_index] = bytes[19];

        Bmp180CalibrationCoefficients.MD_ba[_device_endian_msb_index] = bytes[20];
        Bmp180CalibrationCoefficients.MD_ba[_device_endian_lsb_index] = bytes[21];

        sensor_calibration_read = true;

        return 1;
    }

public:

    Bmp180() = default;

    ~Bmp180() {
        std::cout << "bmp180 destructor called" << std::endl;

        this->_close_device();

        this->run_bmp180_data_capture_thread = false;

        std::unique_lock<std::mutex> data_lock(this->bmp180_data_capture_thread_run_mutex);
        this->bmp180_data_capture_thread_run_cv.notify_one();
        data_lock.unlock();

        if(bmp180_data_capture_thread.joinable()) {
            bmp180_data_capture_thread.join();
        }

        this->mock_run_bmp180_device_thread = false;

        std::unique_lock<std::mutex> device_lock(this->mock_bmp180_device_thread_run_mutex);
        this->mock_bmp180_device_thread_run_cv.notify_one();
        device_lock.unlock();

        if(mock_bmp180_device_thread.joinable()) {
            mock_bmp180_device_thread.join();
        }
    }

    class Addresses {

    public:
        typedef enum CalibrationCoefficients_t {

            AC1 = 0xAAAB,
            AC2 = 0xACAD,
            AC3 = 0xAEAF,
            AC4 = 0xB0B1,
            AC5 = 0xB2B3,
            AC6 = 0xB4B5,
            B1 = 0xB6B7,
            B2 = 0xB8B9,
            MB = 0xBABB,
            MC = 0xBCBD,
            MD = 0xBEBF

        } CalibrationCoefficients;

        typedef enum DataRegisters_t {

            OUTPUT_XLSB = 0xF8,
            OUTPUT_MSB = 0xF6,
            OUTPUT_LSB = 0xF7,
            CONTROL_MEASUREMENT = 0xF4,
            SOFT_RESET = 0xE0,
            CHIP_ID = 0xD0
        } DataRegisters;
    };

    class Commands {

    public:
        typedef enum MeasurementControlValues_t {
            TEMPERATURE = 0x2E,
            PRESSURE_OSS0 = 0x34,
            PRESSURE_OSS1 = 0x74,
            PRESSURE_OSS2 = 0xB4,
            PRESSURE_OSS3 = 0xF4
        } MeasurementControlValues;

        typedef enum SoftReset_t {
            POWER_ON_RESET = 0xB6
        } SoftReset;

        typedef enum ChipId_t {
            CHIP_ID = 0x55
        } ChipId;
    };

    class BitMasks {

    public:
        typedef enum ControlRegister_t {
            OSS_BITS = 0b11000000,
            SCO_BIT = 0b00100000,
            MEASUREMENT_CONTROL_BITS = 0b00011111
        } ControlRegister;
    };

    int config_bmp180(int bus_number, int device_address, int update_period_ms, std::string device_name, bmp180_host_callback_function function_pointer) {
        _bmp180_i2c_bus_number = bus_number;
        _bmp180_i2c_device_address = device_address;
        _bmp180_sensor_update_period_ms = update_period_ms;
        _bmp180_device_name = std::move(device_name);

        _bmp180_host_callback_function = function_pointer;

        bmp180_data_capture_thread = std::thread(&Bmp180::_bmp180_data_capture_worker, this);

        return 0;
    }

    int connect_to_device() {
        int status = 1;

        status &= this->_connect_to_bmp180();

        return status;
    }

    int init_device() {
        this->_init_bmp180();

        return 1;
    }

    int mock_load_bmp180_calibration_data() {

        /*
         * RPI4 is little endian, x86 = little endian  (use lscpu | grep -i byte)
         * https://betterexplained.com/articles/understanding-big-and-little-endian-byte-order/
         */

        /* Memory dump from real device @0x77 on ros-pi-node
        00: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        10: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        20: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        30: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        40: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        50: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        60: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        70: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        80: a5 94 45 09 8d 27 29 3e 7a 55 1e d4 fb b2 c7 27    ??E??')>zU?????'
        90: 83 3b 63 e7 42 65 19 73 00 28 80 00 d1 f6 09 a6    ?;c?Be?s.(?.????
        a0: a5 94 45 09 8d 27 29 3e 7a 55 1e d4 fb b2 c7 27    ??E??')>zU?????'
        b0: 83 3b 63 e7 42 65 19 73 00 28 80 00 d1 f6 09 a6    ?;c?Be?s.(?.????
        c0: 00 00 bc 33 00 00 00 00 00 00 00 10 00 00 00 03    ..?3.......?...?
        d0: 55 02 06 00 00 00 00 00 00 00 00 00 00 00 00 00    U??.............
        e0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00    ................
        f0: 00 00 00 00 00 00 80 00 00 00 00 00 00 00 00 00    ......?.........
         */

        /* Calibration data from real sensor
        d41e b2fb 27c7
        3b83 e763 6542 7319 2800 0080 f6d1 a609
         */


        /*
         * real temp 0x6496 ~ 26C - 78F
         * real pressure 0xa307 - ~ 11PSI, 114F
         */

        uint16_t _mock_device_memory[] = {
                0xDADE, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0xa594, 0x4509, 0x8d27, 0x293e, 0x7a55, 0x1ed4, 0xfbb2, 0xc727,
                0x833b, 0x63e7, 0x4265, 0x1973, 0x0028, 0x8000, 0xd1f6, 0x09a6,
                0xa594, 0x4509, 0x8d27, 0x293e, 0x7a55, 0x1ed4, 0xfbb2, 0xc727,
                0x833b, 0x63e7, 0x4265, 0x1973, 0x0028, 0x8000, 0xd1f6, 0x09a6,
                0x0000, 0xbc33, 0x0000, 0x0000, 0x0000, 0x0010, 0x0000, 0x0003,
                0x5502, 0x0600, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
                0x0000, 0xBABE, 0x0000,
                0xa307, // data word for temp/pressure
                0xDEAF, 0xFFFF, 0xFFFF, 0xFFFF
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

        if (i2c_send(&_bmp180_i2c_context, &outbound_message, register_address)) {
            std::cout << "sent mock calibration data to device OK\n";
            return 0;
        }

        return -1;
    }

    int mock_run_bmp180_device_emulation() {

        mock_bmp180_device_thread = std::thread(&Bmp180::_mock_bmp180_device_emulation, this);

        // wait a little bit for the thread to get started
        std::this_thread::sleep_for(std::chrono::milliseconds (10));

        std::unique_lock<std::mutex> device_lock(this->mock_bmp180_device_thread_run_mutex);
        this->mock_bmp180_device_thread_run_cv.notify_one();
        this->mock_run_bmp180_device_thread = true;
        device_lock.unlock();

        return 1;
    }
};

#endif //BMP180_H
