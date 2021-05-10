//
// Created by user on 4/9/21.
//

#ifndef ADAFRUIT_9D0F_IMU_ADAFRUIT_9D0F_IMU_H
#define ADAFRUIT_9D0F_IMU_ADAFRUIT_9D0F_IMU_H

#include <iostream>
#include <thread>
#include <utility>
#include <condition_variable>

#include "common/i2c/i2c_linux.h"
#include "common/devices/bmp180.h"

class AdaFruit9DoFImu {

private:
    int close_device() {

        i2c_dev_close(&_bmp180_i2c_context, _bmp180_i2c_bus_number);

        return 0;
    }

public:
    AdaFruit9DoFImu() = default;

    ~AdaFruit9DoFImu() {
        std::cout << "destructor called" << std::endl;

        this->close_device();

        this->run_bmp180_data_capture_thread = false;

        if(bmp180_data_capture_thread.joinable()) {
            bmp180_data_capture_thread.join();
        }
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


    /*
     * BMP180
     */
private:
    int _bmp180_i2c_bus_number{};
    int _bmp180_i2c_device_address{};
    std::string _bmp180_device_name;

    context_t _bmp180_i2c_context{};

    Bmp180 bmp180;
    uint8_t _bmp180_calibration_data_buffer[11 * 2] = {0};
    uint16_t _bmp180_long_uncompensated_temperature{};
    uint16_t _bmp180_long_uncompensated_pressure{};
    uint8_t  _bmp180_short_uncompensated_pressure_xlsb{};

    typedef void (*bmp180_callback_function)(int);
    bmp180_callback_function _bmp180_callback_function{};

    bool run_bmp180_data_capture_thread = false;
    std::condition_variable bmp180_data_capture_thread_run_cv;
    std::mutex bmp180_data_capture_thread_run_mutex;
    std::thread bmp180_data_capture_thread;

    void _bmp180_data_capture_worker();

    int _init_bmp180() {

        buffer_t inbound_message = {
                .bytes = _bmp180_calibration_data_buffer,
                .size = sizeof(_bmp180_calibration_data_buffer)
        };

        uint8_t register_address = (Bmp180::Addresses::CalibrationCoefficients::AC1 >> 8) & 0xff;
        if (i2c_recv(&_bmp180_i2c_context, &inbound_message, register_address)) {

            std::cout << "calibration data: " << std::endl;

            for(int i = 0; i < sizeof(_bmp180_calibration_data_buffer); ++i) {
                std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)_bmp180_calibration_data_buffer[i] << " ";
            }

            std::cout << std::endl;
        }

        this->bmp180.init_bmp180_calibration_coefficients((char *) this->_get_bmp180_calibration_buffer_address(),
                                                          this->_get_bmp180_calibration_buffer_size());

        std::lock_guard<std::mutex> lk(this->bmp180_data_capture_thread_run_mutex);
        this->bmp180_data_capture_thread_run_cv.notify_one();
        this->run_bmp180_data_capture_thread = true;

        return 1;
    }

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

        return 1;
    }

    void _get_bmp180_temperature();

    void _get_bmp180_pressure();

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

public:
    int config_bmp180(int bus_number, int device_address, std::string device_name, bmp180_callback_function function_pointer) {
        _bmp180_i2c_bus_number = bus_number;
        _bmp180_i2c_device_address = device_address;
        _bmp180_device_name = std::move(device_name);

        _bmp180_callback_function = function_pointer;

        bmp180_data_capture_thread = std::thread(&AdaFruit9DoFImu::_bmp180_data_capture_worker, this);

        bmp180 = Bmp180();

        return 0;
    }

    int load_mock_bmp180_calibration_data() {

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
         * real temp 0x6496
         * real pressure 0xa307 0x00
         */

        static uint16_t _mock_device_memory[] = {
                0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
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
                0x0000, 0x0000, 0x0000, 0x6496, 0x0000, 0x0000, 0x0000, 0x0000
        };


        uint8_t send_buffer[sizeof(_mock_device_memory)];
        std::memset(&send_buffer, 0, sizeof(send_buffer));

        std::memcpy(&send_buffer, _mock_device_memory, sizeof(_mock_device_memory));

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


    /*
     * L3GD20
     */
private:
    int _l3gd20_i2c_bus_number{};
    int _l3gd20_i2c_device_address{};
    std::string _l3gd20_device_name;

    context_t _l3gd20_i2c_context{};

    int _init_l3gd20() {

        return 0;
    }

    int _connect_to_l3gd20() {
        return 0;
    }

public:

    int load_mock_l3gd20_data() {
        return 0;
    }


    /*
     * LSM303DLHC
     */
private:
    int _lsm303dlhc_i2c_bus_number{};
    int _lsm303dlhc_i2c_device_address{};
    std::string _lsm303dlhc_device_name;

    context_t _lsm303dlhc_i2c_context{};

    int _init_lsm303dlhc() {

        return 0;
    }

    int _connect_to_lsm303dlhc() {
        return 0;
    }

public:

    int load_mock_lsm303dlhc_data() {
        return 0;
    }
};

#endif //ADAFRUIT_9D0F_IMU_ADAFRUIT_9D0F_IMU_H
