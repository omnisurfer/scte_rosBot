//
// Created by user on 4/9/21.
//
#include <iostream>
#include <iomanip>
#include <string>

#include <chrono>
#include <thread>
#include <cstring>

#include "common/i2c/i2c_linux.h"
#include "common/devices/bmp180.h"

#include "adafruit_9d0f_imu.h"


void AdaFruit9DoFImu::read_temperature() {
// command temperature read
    // notify bmp180 to read temp reg
    // wait 4.5ms
    // read reg 0xF6F7
    uint8_t control_register_and_start_meas[1] = {0x2E};
    buffer_t outbound_message = {
            .bytes = control_register_and_start_meas,
            .size = 1
    };

    uint8_t register_address;
    buffer_t inbound_message;

    register_address = Bmp180::Addresses::DataRegisters::CONTROL_MEASUREMENT;
    if (i2c_send(&_ada_i2c_context, &outbound_message, register_address)) {
        std::cout << "sent buffer to device OK\n";
        std::this_thread::sleep_for(std::chrono::microseconds (4500));
    }

    // read back temperature
    uint8_t uncompensated_temperature[2] = {0};
    inbound_message = {
            .bytes = uncompensated_temperature,
            .size = sizeof(uncompensated_temperature)
    };

    register_address = (Bmp180::Addresses::DataRegisters::OUTPUT >> 8) & 0xff;
    i2c_recv(&_ada_i2c_context, &inbound_message, register_address);

    _long_uncompensated_temperature = (uncompensated_temperature[1] << 8) | uncompensated_temperature[0];
}

void AdaFruit9DoFImu::read_pressure() {
    uint8_t control_register_and_start_meas[1] = {0x2E};
    buffer_t outbound_message = {
            .bytes = control_register_and_start_meas,
            .size = 1
    };

    uint8_t register_address;
    buffer_t inbound_message;

    // command pressure read
    // write 0x34 + (oss << 6) to 0xF4
    // wait 4.5ms
    //read 0xF6, 0xF7, 0xF8
    uint8_t oss_value = 0;
    control_register_and_start_meas[0] = {static_cast<uint8_t>(0x34 + (oss_value << 6))};
    outbound_message = {
            .bytes = control_register_and_start_meas,
            .size = 1
    };

    register_address = Bmp180::Addresses::DataRegisters::CONTROL_MEASUREMENT;
    if (i2c_send(&_ada_i2c_context, &outbound_message, register_address)) {
        std::cout << "sent buffer to device OK\n";
        std::this_thread::sleep_for(std::chrono::microseconds (4500));
    }

    // read back pressure
    uint8_t uncompensated_pressure[3] = {0};
    inbound_message = {
            .bytes = uncompensated_pressure,
            .size = sizeof(uncompensated_pressure)
    };

    register_address = (Bmp180::Addresses::DataRegisters::OUTPUT >> 8) & 0xff;
    i2c_recv(&_ada_i2c_context, &inbound_message, register_address);

    _long_uncompensated_pressure = (uncompensated_pressure[0] << 8) | uncompensated_pressure[1];
    _short_uncompensated_pressure_xlsb = uncompensated_pressure[2];


}

void AdaFruit9DoFImu::data_capture_worker() {
    std::cout << "data_capture_worker starting" << std::endl;

    /*
    std::unique_lock<std::mutex> lk(this->data_capture_thread_access_mutex);
    std::cout << "waiting to run..." << std::endl;
    this->data_capture_thread_cv.wait(lk, [this]{
        return run_data_capture_thread;
    });
    */

    /**/
    while(!this->run_data_capture_thread) {
        std::cout << "waiting to run..." << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    /**/

    while(this->run_data_capture_thread) {

        this->read_temperature();

        this->read_pressure();

        uint16_t long_uncompensated_temperature = this->get_uncompensated_temperature_count();
        uint16_t long_uncompensated_pressure = this->get_uncompensated_pressure();
        uint8_t short_uncompensated_pressure_xlsb = this->get_uncompensated_pressure_xlsb();

        /*
         * Start calculating measurements with the Bmp180 code
         */
        Bmp180 bmp180 = Bmp180();
        bmp180.init_bmp180_calibration_coefficients((char *) this->get_calibration_buffer_address(),
                                                    this->get_calibration_buffer_size());

        float calculated_temperature = 0.0f;
        bmp180.calculate_temperature(long_uncompensated_temperature, calculated_temperature);

        float calculated_pressure = 0.0f;
        bmp180.calculate_pressure(long_uncompensated_pressure, short_uncompensated_pressure_xlsb, calculated_pressure);

        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "data_capture_worker exiting" << std::endl;
}

void handle_measurements(int some_int) {
    std::cout << "got called to handle data " << some_int << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "Hello World adafruit 9dof IMU" << std::endl;

    int i2c_bus_number = 0;
    int i2c_device_address = 0x03;

    // lame way to do this but good enough for debug
    if(argv[1]) {
        if (!memcmp("-d", argv[1], 2)) {
            // i2c_bus_number = atoi(argv[2]);

            char* p_end;
            i2c_bus_number = (int)std::strtol(argv[2], &p_end, 10);

            if (*p_end) {
                //not sure what to do in this case
            }
        }
    }

    if(argv[3]) {
        if(!memcmp("-a", argv[3], 2)) {
            i2c_device_address = std::stoi(argv[4], 0, 16);
        }
    }

    AdaFruit9DoFImu adaFruit9DoFImu = AdaFruit9DoFImu(
            i2c_bus_number,
            i2c_device_address,
            "pressure sensor",
            &handle_measurements
            );

    int connect_ok = adaFruit9DoFImu.connect_to_device();

    if(!connect_ok) {
        return 0;
    }

    adaFruit9DoFImu.load_mock_calibration_data();

    adaFruit9DoFImu.init_device();

    std::cout << "press any key to continue..." << std::endl;

    std::cin.get();

    std::cout << "exiting thread" << std::endl;

    return 0;
}