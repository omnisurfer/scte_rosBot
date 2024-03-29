//
// Created by user on 4/9/21.
//

#ifndef BMP180_H
#define BMP180_H

// some code taken from my github:
// https://github.com/omnisurfer/IMUController_firmware_AtmelC/blob/master/IMU/src/app_components/sensors/bmp180/bmp180.c

#include <iostream>
#include <iomanip>
#include <cstring>
#include <thread>
#include <condition_variable>

#include "i2c_linux/i2c_linux.h"
#include "utils/boost_logging.h"
#include "utils/register_utils.h"

class Bmp180Pressure {

private:

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

            OUTPUT_MSB = 0xF6,
            OUTPUT_LSB = 0xF7,
            OUTPUT_XLSB = 0xF8,
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
    };

    class MagicNumbers {
    public:
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

    std::mutex _i2c_device_mutex;
    int _i2c_bus_number{};
    int _i2c_device_address{};
    int _sensor_update_period_ms{};
    std::string _device_name;

    context_t _i2c_device_context{};

    bool _enable_load_mock_data = false;

    uint8_t _calibration_data_buffer[11 * 2] = {0};
    uint16_t _long_uncompensated_temperature{};
    uint16_t _long_uncompensated_pressure{};
    uint8_t  _short_uncompensated_pressure_xlsb{};

    bool run_data_capture_thread = false;
    bool is_running_data_capture_thread = false;
    std::condition_variable data_capture_thread_run_cv;
    std::mutex data_capture_thread_run_mutex;
    std::thread data_capture_thread;

    typedef void (*host_callback_function)(
            float calculated_temperature, float calculated_pressure
            );

    host_callback_function _host_callback_function{};

    bool run_mock_device_thread = false;
    bool is_running_mock_device_thread = false;
    std::condition_variable mock_device_thread_run_cv;
    std::mutex mock_device_thread_run_mutex;
    std::thread mock_device_thread;

    std::mutex& _data_capture_worker_execute_cycle_mutex;
    std::condition_variable& _data_capture_worker_execute_cycle_conditional_variable;

    bool sensor_calibration_read = false;

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

    } Bmp180CalibrationCoefficients{};

    struct Bmp180SharedCoefficients_t {
        union {
            long B5;
        };
    } Bmp180SharedCoefficients{};

    int _connect_to_device() {

        // set up the i2c context and connect
        _i2c_device_context = {0};
        if(!open_i2c_dev(_i2c_bus_number, _i2c_device_address)) {
            BOOST_LOG_TRIVIAL(error) << this->_device_name << ": failed to open device";
            return 0;
        }

        if(!is_i2c_dev_connected()) {
            BOOST_LOG_TRIVIAL(error) << this->_device_name << ": failed to connect to device";
            return 0;
        }

        if(_enable_load_mock_data) {
            _mock_load_calibration_data();
        }

        // try read chip id
        uint8_t chip_id[1] = {0};
        buffer_t inbound_message = {
                .bytes = chip_id,
                .size = sizeof(chip_id)
        };

        uint8_t register_address;
        register_address = Bmp180Pressure::Addresses::DataRegisters::CHIP_ID;
        receive_i2c(&inbound_message, register_address);

        if(chip_id[0] != Bmp180Pressure::MagicNumbers::ChipId::CHIP_ID) {
            BOOST_LOG_TRIVIAL(error) << this->_device_name << ": failed to read device chip id";
            return 0;
        }

        return 1;
    }

    int _init_device();

    int _close_device() {

        close_i2c_dev(_i2c_bus_number);

        return 0;
    }

    int receive_i2c(buffer_t *data, uint8_t register_address);
    int send_i2c(buffer_t *data, uint8_t register_address);
    int open_i2c_dev(int device_number, int slave_address);
    int is_i2c_dev_connected();
    void close_i2c_dev(int bus_number);

    int _mock_load_calibration_data() {

        /*
         * RPI4 is little endian, x86 = little endian  (use lscpu | grep -i byte)
         * https://betterexplained.com/articles/understanding-big-and-little-endian-byte-order/
         */

        /* Memory dump from real device @0x77 on ros-pi-node - Idle?
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

        buffer_t outbound_message = {
                .bytes = send_buffer,
                .size = sizeof(send_buffer)
        };

        int8_t register_address = 0x00;

        if (send_i2c(&outbound_message, register_address)) {
            BOOST_LOG_TRIVIAL(debug) << this->_device_name << ": sent mock calibration data to device OK";
            return 0;
        }

        return -1;
    }

    void _data_capture_worker();

    void _mock_device_emulation_worker();

    void _request_temperature();

    void _request_pressure();

    int _measurement_completed_ok();

    uint16_t _get_uncompensated_temperature_count() const {
        return _long_uncompensated_temperature;
    }

    uint16_t _get_uncompensated_pressure() const {
        return _long_uncompensated_pressure;
    }

    uint8_t _get_uncompensated_pressure_xlsb() const {
        return _short_uncompensated_pressure_xlsb;
    }

    uint8_t* _get_calibration_buffer_address() {
        return _calibration_data_buffer;
    }

    int _get_calibration_buffer_size() {
        return sizeof(_calibration_data_buffer);
    }

    // Note, temperature_deg_c must be read first, then pressure so that B5 value can be populated
    int _calculate_temperature(uint16_t uncompensated_temperature, float &temperature) {

        if(!sensor_calibration_read) {
            BOOST_LOG_TRIVIAL(debug) << this->_device_name << ": sensor calibration not read yet";
            return 0;
        }

        // refer to formula found on page 15 of datasheet
        long _temperature = uncompensated_temperature;

        float FX1 = float(_temperature - Bmp180CalibrationCoefficients.AC6) * (float(Bmp180CalibrationCoefficients.AC5) / 32768);
        float FX2 = float(Bmp180CalibrationCoefficients.MC * 2048) / (FX1 + float(Bmp180CalibrationCoefficients.MD));

        long X1 = long(FX1);
        long X2 = long(FX2);

        Bmp180SharedCoefficients.B5 = X1 + X2;

        _temperature =  (Bmp180SharedCoefficients.B5 + 8) / 16;

        // multiplying by 0.1 here to convert to degrees C instead of degrees 0.1C
        temperature = (float)_temperature * 0.1f;

        //std::cout << "calc temp (C): " << temperature_deg_c << std::endl;

        return 1;

    }

    int _calculate_pressure(uint16_t uncompensated_pressure, uint8_t uncompensated_pressure_xlsb, float &pressure) const {

        if(!sensor_calibration_read) {
            BOOST_LOG_TRIVIAL(debug) << this->_device_name << ": sensor calibration not read yet";
            return 0;
        }

        uint8_t oss_temp = 0;

        // uint8_t ut_l = 0x23;
        // uint8_t ut_h = 0x5d;

        // long _pressure = ((ut_h << 16) + (ut_l << 8) + uncompensated_pressure_xlsb) >> (8 - 0); 0 is OSS@0
        long _pressure = (uncompensated_pressure << 8) >> (8 - oss_temp);
        _pressure += uncompensated_pressure_xlsb;

        // refer to formula found on page 15 of datasheet
        // all these operations in powers of 2 seem to be "hiding" bit-shifts. wonder if the compiler will notice
        // this...
        long _p = 0,
                B6 = Bmp180SharedCoefficients.B5 - 4000,
                X1 = (Bmp180CalibrationCoefficients.B2 * ((B6 * B6) / 4096)) / 2048,
                X2 = (Bmp180CalibrationCoefficients.AC2 * B6) / 2048,
                X3 = X1 + X2,
                B3 = (((Bmp180CalibrationCoefficients.AC1 * 4 + X3) << oss_temp) + 2) / 4;

        X1 = (Bmp180CalibrationCoefficients.AC3 * B6) / 8192;
        X2 = (Bmp180CalibrationCoefficients.B1 * (B6 * B6 / 4096)) / 65536;
        X3 = ((X1 + X2) + 2) / 4;

        unsigned long B4 = Bmp180CalibrationCoefficients.AC4 * (unsigned long)(X3 + 32768) / 32768;

        //note _B7 has a scaling option that is tied to the oversample (OSS). I am assuming 0 oversampling for now
        unsigned long B7 = ((unsigned long)_pressure - B3) * (50000 >> oss_temp);

        if(B7 < 0x80000000) {
            _p = (B7 * 2) / B4;
        }
        else {
            _p = (B7 / B4) * 2;
        }

        X1 = ( _p/256) * (_p/256);
        X1 = (X1 * 3038) / 65536;
        X2 = (-7357 * _p) / 65536;

        _p = _p + ((X1 + X2 + 3791) / 16);

        pressure = (float)_p;

        //std::cout << "calc _p: " << _p << std::endl;

        return 1;
    }

    int _init_calibration_coefficients(char *bytes, uint8_t length) {
        BOOST_LOG_TRIVIAL(debug) << this->_device_name << ": _init_calibration_coefficients";

        if(length < 0 or length > 22) {
            return 0;
        }

        // device seems to be big endian
        // MSB 0xAA
        // LSB 0xAB
        Bmp180CalibrationCoefficients.AC1_ba[1] = bytes[0];
        Bmp180CalibrationCoefficients.AC1_ba[0] = bytes[1];

        // MSB 0xAC
        // LSB 0xAD
        Bmp180CalibrationCoefficients.AC2_ba[1] = bytes[2];
        Bmp180CalibrationCoefficients.AC2_ba[0] = bytes[3];

        // MSB 0xAE
        // LSB 0xAF
        Bmp180CalibrationCoefficients.AC3_ba[1] = bytes[4];
        Bmp180CalibrationCoefficients.AC3_ba[0] = bytes[5];

        // MSB 0xB0
        // LSB 0xB1
        Bmp180CalibrationCoefficients.AC4_ba[1] = bytes[6];
        Bmp180CalibrationCoefficients.AC4_ba[0] = bytes[7];

        // MSB 0xB2
        // LSB 0xB3
        Bmp180CalibrationCoefficients.AC5_ba[1] = bytes[8];
        Bmp180CalibrationCoefficients.AC5_ba[0] = bytes[9];

        // MSB 0xB4
        // LSB 0xB5
        Bmp180CalibrationCoefficients.AC6_ba[1] = bytes[10];
        Bmp180CalibrationCoefficients.AC6_ba[0] = bytes[11];

        // MSB 0xB6
        // LSB 0xB7
        Bmp180CalibrationCoefficients.B1_ba[1] = bytes[12];
        Bmp180CalibrationCoefficients.B1_ba[0] = bytes[13];

        // MSB 0xB8
        // LSB 0xB9
        Bmp180CalibrationCoefficients.B2_ba[1] = bytes[14];
        Bmp180CalibrationCoefficients.B2_ba[0] = bytes[15];

        // MSB 0xBA
        // LSB 0xBB
        Bmp180CalibrationCoefficients.MB_ba[1] = bytes[16];
        Bmp180CalibrationCoefficients.MB_ba[0] = bytes[17];

        // MSB 0xBC
        // LSB 0xBD
        Bmp180CalibrationCoefficients.MC_ba[1] = bytes[18];
        Bmp180CalibrationCoefficients.MC_ba[0] = bytes[19];

        // MSB 0xBE
        // LSB 0xBF
        Bmp180CalibrationCoefficients.MD_ba[1] = bytes[20];
        Bmp180CalibrationCoefficients.MD_ba[0] = bytes[21];

        sensor_calibration_read = true;

        return 1;
    }

public:

    //TODO Correct variance calculation TBD
    /* from datasheet */
    constexpr static const double relative_accuracy_pressure_hpa = 0.12;
    constexpr static const double rms_noise_pressure_hpa = 0.05;
    constexpr static const double absolute_accuracy_temperature = 2.0;

    /* probably the wrong values for this matrix */
    constexpr static const double pressure_variance = relative_accuracy_pressure_hpa;
    constexpr static const double temperature_variance = absolute_accuracy_temperature;

    //Bmp180Pressure() = default;
    Bmp180Pressure(std::mutex& worker_execute_mutex, std::condition_variable& worker_execute_conditional_variable):
            _data_capture_worker_execute_cycle_mutex(worker_execute_mutex),
            _data_capture_worker_execute_cycle_conditional_variable(worker_execute_conditional_variable){
    }

    ~Bmp180Pressure() {

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
        _device_name = std::move(device_name);

        _host_callback_function = function_pointer;

        data_capture_thread = std::thread(&Bmp180Pressure::_data_capture_worker, this);

        return 0;
    }

    int connect_to_device() {
        int status = 1;

        status &= this->_connect_to_device();

        return status;
    }

    int init_device(int update_period_ms) {

        _sensor_update_period_ms = update_period_ms;

        this->_init_device();

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
            if (data_capture_thread.joinable()) {
                data_capture_thread.join();
            }
        }

        BOOST_LOG_TRIVIAL(debug) << this->_device_name << ": data_capture_thread joined";

        bool mock_device_thread_was_running = false;
        std::unique_lock<std::mutex> mock_device_lock(this->mock_device_thread_run_mutex);
        {
            if(is_running_mock_device_thread) {
                mock_device_thread_was_running = is_running_mock_device_thread;
                this->run_mock_device_thread = false;
                this->mock_device_thread_run_cv.notify_one();
            }
            mock_device_lock.unlock();
        }

        if(mock_device_thread_was_running) {
            if (mock_device_thread.joinable()) {
                mock_device_thread.join();
            }
        }

        BOOST_LOG_TRIVIAL(debug) << this->_device_name << ": mock_device_thread joined";

        this->_close_device();
    }

    void enable_load_mock_data();

    int mock_run_device_emulation() {

        mock_device_thread = std::thread(&Bmp180Pressure::_mock_device_emulation_worker, this);

        // wait a little for the thread to get started
        std::this_thread::sleep_for(std::chrono::milliseconds (10));

        std::unique_lock<std::mutex> device_lock(this->mock_device_thread_run_mutex);
        this->run_mock_device_thread = true;
        this->mock_device_thread_run_cv.notify_one();
        device_lock.unlock();

        return 1;
    }
};

#endif //BMP180_H

