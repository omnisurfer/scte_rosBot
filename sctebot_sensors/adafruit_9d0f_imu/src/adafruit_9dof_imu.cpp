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

class AdaFruit9DoFImu {
public:
    int my_num;
    std::string name_of_device;
};


void Bmp180WriteMockCoefficientsToMemory(char* mock_coefficients, int16_t array_size) {

    context_t ada_i2c_context = {0};
    int device_id = 0;
    int slave_id = 0x03;

    if(!i2c_dev_open(&ada_i2c_context, device_id, slave_id)) {
        std::cout << "failed to open device\n";
    }

    if(!i2c_is_connected(&ada_i2c_context)) {
        std::cout << "failed to connect to device\n";
    }

    uint8_t send_buffer[array_size];
    std::memset(&send_buffer, 0, sizeof(send_buffer));

    std::memcpy(&send_buffer, mock_coefficients, array_size);

    buffer_t outbound_message = {
            .bytes = send_buffer,
            .size = sizeof(send_buffer)
    };

    int8_t register_address = 0x00;

    if (i2c_send(&ada_i2c_context, &outbound_message, register_address)) {
        std::cout << "sent buffer to device OK\n";
    }

    i2c_dev_close(&ada_i2c_context, device_id);
}


int main(int argc, char* argv[]) {
    std::cout << "Hello World adafruit 9dof IMU" << std::endl;

    int i2c_device_number = 0;
    int device_address = 0x03;

    // lame way to do this but good enough for debug
    if(argv[1]) {
        if (!memcmp("-d", argv[1], 2)) {
            i2c_device_number = atoi(argv[2]);
        }
    }

    if(argv[3]) {
        if(!memcmp("-a", argv[3], 2)) {
            device_address = std::stoi(argv[4], 0, 16);
        }
    }

    /*
     * ARM = Big endian, x86 = little endian
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

    uint16_t actual_device_memory[] = {
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

    /*
     * real temp 0x6496
     * real pressure 0xa307 0x00
     */

    uint16_t mock_bmp180_uncompensated_temp = 0x6496;
    uint16_t mock_bmp180_uncompensated_pressure = 0xa307;
    uint8_t mock_bmp180_uncompensated_pressure_xlsb = 0x00;

    /*
     * setup the i2c context and connect
     */
    context_t ada_i2c_context = {0};
    int device_id = i2c_device_number;
    int slave_id = device_address;

    if(!i2c_dev_open(&ada_i2c_context, device_id, slave_id)) {
        std::cout << "failed to open device\n";
        return -1;
    }

    if(!i2c_is_connected(&ada_i2c_context)) {
        std::cout << "failed to connect to device\n";
        return -1;
    }

    /* Write out mock data into the mock device */
    Bmp180WriteMockCoefficientsToMemory((char *) actual_device_memory, sizeof(actual_device_memory));

    /*
     * prepare to read the params and data
     */
    uint8_t calibration_data_buffer[11 * 2] = {0};

    buffer_t inbound_message = {
            .bytes = calibration_data_buffer,
            .size = sizeof(calibration_data_buffer)
    };

    uint8_t register_address = (Bmp180::Addresses::CalibrationCoefficients::AC1 >> 8) & 0xff;
    if (i2c_recv(&ada_i2c_context, &inbound_message, register_address)) {

        std::cout << "calibration data: " << std::endl;

        for(int i = 0; i < sizeof(calibration_data_buffer); ++i) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)calibration_data_buffer[i] << " ";
        }

        std::cout << std::endl;
    }

    /* Calibration data from real sensor
    d41e b2fb 27c7
    3b83 e763 6542 7319 2800 0080 f6d1 a609
     */

    // command temperature read
    // notify bmp180 to read temp reg
    // wait 4.5ms
    // read reg 0xF6F7
    uint8_t control_register_and_start_meas[1] = {0x2E};
    buffer_t outbound_message = {
            .bytes = control_register_and_start_meas,
            .size = 1
    };

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
            static_cast<uint8_t>((mock_bmp180_uncompensated_pressure) & 0xff),
            static_cast<uint8_t>((mock_bmp180_uncompensated_pressure >> 8) & 0xff)
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

    uint16_t long_uncompensated_pressure = (uncompensated_pressure[2] << 8) | uncompensated_pressure[1];
    uint8_t  short_uncompensated_pressure_xlsb = uncompensated_pressure[0];

    i2c_dev_close(&ada_i2c_context, device_id);

    /*
     * Start calculating measurements with the Bmp180 code
     */
    Bmp180 bmp180 = Bmp180();
    bmp180.init_bmp180_calibration_coefficients((char *)calibration_data_buffer, sizeof(calibration_data_buffer));

    float calculated_temperature = 0.0f;
    bmp180.calculate_temperature(long_uncompensated_temperature, calculated_temperature);

    float calculated_pressure = 0.0f;
    bmp180.calculate_pressure(long_uncompensated_pressure, short_uncompensated_pressure_xlsb, calculated_pressure);

    return 0;
}