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
    int _i2c_bus_number;
    int _i2c_device_address;
    std::string _device_name;

    context_t _ada_i2c_context;

    uint8_t _calibration_data_buffer[11 * 2] = {0};

    uint16_t _long_uncompensated_temperature;

    uint16_t _long_uncompensated_pressure;
    uint8_t  _short_uncompensated_pressure_xlsb;

    typedef void (*client_callback_function)(int);
    client_callback_function _client_callback_function;

    bool run_data_capture_thread = false;
    //std::condition_variable data_capture_thread_cv;
    //std::mutex data_capture_thread_access_mutex;
    std::thread data_capture_thread;

    void data_capture_worker();

    int close_device() {

        i2c_dev_close(&_ada_i2c_context, _i2c_bus_number);

        return 0;
    }

public:
    AdaFruit9DoFImu(int bus_number, int device_address, std::string device_name, client_callback_function function_pointer) {
        _i2c_bus_number = bus_number;
        _i2c_device_address = device_address;
        _device_name = std::move(device_name);

        _client_callback_function = function_pointer;

        data_capture_thread = std::thread(&AdaFruit9DoFImu::data_capture_worker, this);
    }

    /*
     * https://stackoverflow.com/questions/64650981/call-to-implicitly-deleted-copy-constructor-of-class-error
     */
    AdaFruit9DoFImu(const AdaFruit9DoFImu&) = delete;
    AdaFruit9DoFImu& operator=(const AdaFruit9DoFImu&) = delete;

    AdaFruit9DoFImu(AdaFruit9DoFImu&&) noexcept = default;
    AdaFruit9DoFImu& operator=(AdaFruit9DoFImu&&) noexcept = default;

    ~AdaFruit9DoFImu() {
        std::cout << "destructor called" << std::endl;

        this->close_device();

        this->run_data_capture_thread = false;

        if(data_capture_thread.joinable()) {
            data_capture_thread.join();
        }
    }

    int connect_to_device() {

        /*
         * setup the i2c context and connect
         */
        _ada_i2c_context = {0};
        if(!i2c_dev_open(&_ada_i2c_context, _i2c_bus_number, _i2c_device_address)) {
            std::cout << "failed to open device\n";
            return 0;
        }

        if(!i2c_is_connected(&_ada_i2c_context)) {
            std::cout << "failed to connect to device\n";
            return 0;
        }

        return 1;
    }

    int load_mock_calibration_data() {

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

        if (i2c_send(&_ada_i2c_context, &outbound_message, register_address)) {
            std::cout << "sent mock calibration data to device OK\n";
            return 0;
        }

        return -1;
    }

    int init_device() {

        buffer_t inbound_message = {
                .bytes = _calibration_data_buffer,
                .size = sizeof(_calibration_data_buffer)
        };

        uint8_t register_address = (Bmp180::Addresses::CalibrationCoefficients::AC1 >> 8) & 0xff;
        if (i2c_recv(&_ada_i2c_context, &inbound_message, register_address)) {

            std::cout << "calibration data: " << std::endl;

            for(int i = 0; i < sizeof(_calibration_data_buffer); ++i) {
                std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)_calibration_data_buffer[i] << " ";
            }

            std::cout << std::endl;
        }

        //start the thread
        this->run_data_capture_thread = true;

        return 1;
    }

    void read_temperature();

    void read_pressure();

    uint16_t get_uncompensated_temperature_count() const {
        return _long_uncompensated_temperature;
    }

    uint16_t get_uncompensated_pressure() const {
        return _long_uncompensated_pressure;
    }

    uint8_t get_uncompensated_pressure_xlsb() const {
        return _short_uncompensated_pressure_xlsb;
    }

    uint8_t* get_calibration_buffer_address() {
        return _calibration_data_buffer;
    }

    int get_calibration_buffer_size() {
        return sizeof(_calibration_data_buffer);
    }
};

#endif //ADAFRUIT_9D0F_IMU_ADAFRUIT_9D0F_IMU_H
