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

}

void AdaFruit9DoFImu::read_pressure() {

}

int main(int argc, char* argv[]) {
    std::cout << "Hello World adafruit 9dof IMU" << std::endl;

    int i2c_bus_number = 0;
    int i2c_device_address = 0x03;

    // lame way to do this but good enough for debug
    if(argv[1]) {
        if (!memcmp("-d", argv[1], 2)) {
            i2c_bus_number = atoi(argv[2]);
        }
    }

    if(argv[3]) {
        if(!memcmp("-a", argv[3], 2)) {
            i2c_device_address = std::stoi(argv[4], 0, 16);
        }
    }

    AdaFruit9DoFImu adaFruit9DoFImu = AdaFruit9DoFImu(i2c_bus_number, i2c_device_address, "pressure sensor");

    adaFruit9DoFImu.start_demo_thread();

    uint16_t mock_bmp180_uncompensated_temp = 0x6496;
    uint16_t mock_bmp180_uncompensated_pressure = 0xa307;
    uint8_t mock_bmp180_uncompensated_pressure_xlsb = 0x00;

    adaFruit9DoFImu.connect_to_device();

    adaFruit9DoFImu.load_mock_calibration_data();

    adaFruit9DoFImu.init_device();

    /*
     * setup the i2c context and connect
     */
    context_t ada_i2c_context = {0};
    int device_id = i2c_bus_number;
    int slave_id = i2c_device_address;

    if(!i2c_dev_open(&ada_i2c_context, device_id, slave_id)) {
        std::cout << "failed to open device\n";
        return -1;
    }

    if(!i2c_is_connected(&ada_i2c_context)) {
        std::cout << "failed to connect to device\n";
        return -1;
    }

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
    if (i2c_send(&ada_i2c_context, &outbound_message, register_address)) {
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
    i2c_recv(&ada_i2c_context, &inbound_message, register_address);

    uint16_t long_uncompensated_temperature = (uncompensated_temperature[1] << 8) | uncompensated_temperature[0];

    //DEBUG SET MOCK PRESSURE
    uint8_t mock_pressure[2] = {
            static_cast<uint8_t>((mock_bmp180_uncompensated_pressure >> 8) & 0xff),
            static_cast<uint8_t>((mock_bmp180_uncompensated_pressure >> 0) & 0xff)
    };

    outbound_message = {
            .bytes = mock_pressure,
            .size = sizeof(mock_pressure)
    };
    //DEBUG SET MOCK PRESSURE

    register_address = 0xF6;
    if (i2c_send(&ada_i2c_context, &outbound_message, register_address)) {
        std::cout << "sent buffer to device OK\n";
        std::this_thread::sleep_for(std::chrono::microseconds (4500));
    }

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
    if (i2c_send(&ada_i2c_context, &outbound_message, register_address)) {
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
    i2c_recv(&ada_i2c_context, &inbound_message, register_address);

    uint16_t long_uncompensated_pressure = (uncompensated_pressure[0] << 8) | uncompensated_pressure[1];
    uint8_t  short_uncompensated_pressure_xlsb = uncompensated_pressure[2];

    i2c_dev_close(&ada_i2c_context, device_id);

    /*
     * Start calculating measurements with the Bmp180 code
     */
    Bmp180 bmp180 = Bmp180();
    // bmp180.init_bmp180_calibration_coefficients((char *)calibration_data_buffer, sizeof(calibration_data_buffer));
    bmp180.init_bmp180_calibration_coefficients((char *)adaFruit9DoFImu.get_calibration_buffer_address(),
                                                adaFruit9DoFImu.get_calibration_buffer_size());

    float calculated_temperature = 0.0f;
    bmp180.calculate_temperature(long_uncompensated_temperature, calculated_temperature);

    float calculated_pressure = 0.0f;
    bmp180.calculate_pressure(long_uncompensated_pressure, short_uncompensated_pressure_xlsb, calculated_pressure);

    return 0;
}