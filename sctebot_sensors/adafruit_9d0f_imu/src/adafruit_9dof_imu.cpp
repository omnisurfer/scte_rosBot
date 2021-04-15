//
// Created by user on 4/9/21.
//
#include <iostream>
#include <string>

#include <chrono>
#include <thread>

#include "common/i2c/i2c_linux.h"
#include "common/devices/bmp180.h"

class AdaFruit9DoFImu {
public:
    int myNum;
    std::string nameOfDevice;
};

int main() {
    std::cout << "Hello World adafruit" << std::endl;

    Bmp180 bmp180 = Bmp180();

    uint16_t test = Bmp180::Addresses::CalibrationCoefficients_t::AC1;
    uint8_t abc = Bmp180::Commands::MeasurementControl::PRESSURE_OSS0;

    uint8_t test_coeff[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A,
                            0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x13, 0x14};

    int16_t example_calib_coeff[] = {
            408,
            -72,
            -14383,
            32741,
            32757,
            23153,
            6190,
            4,
            -32768,
            -8711,
            2868
    };

    bmp180.init_bmp180_calibration_coefficients((char *)example_calib_coeff, sizeof(example_calib_coeff));

    uint8_t ut_l = 0xfa;
    uint8_t ut_h = 0x6c;

    uint16_t uncompensated_temp = 27898;
    uint16_t uncompensated_pressure = 23843;
    uint8_t uncompensated_pressure_xlsb = 0xF8;

    float temp = 0.0f;
    bmp180.calculate_temperature(uncompensated_temp, temp);

    float pressure = 0.0f;
    bmp180.calculate_pressure(uncompensated_pressure, uncompensated_pressure_xlsb,pressure);

    context_t ada_i2c_context = {0};
    int device_id = 0;

    uint8_t receiveBuffer[MAX_BUFFER_SIZE] = {0};
    uint8_t sendBuffer[] = "Hello Adafruit 9dof";

    if(!i2c_dev_open(&ada_i2c_context, device_id, 0x03)) {
        std::cout << "failed to open device\n";
    }

    if(!i2c_is_connected(&ada_i2c_context)) {
        std::cout << "failed to connect to device\n";
    }

    buffer_t outboundMessage = {
            .bytes = sendBuffer,
            .size = sizeof(sendBuffer)
    };

    const std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();

    std::chrono::time_point<std::chrono::steady_clock> start = std::chrono::steady_clock::now();

    for(int i = 0; i < 100; i++) {
        if (i2c_send(&ada_i2c_context, &outboundMessage)) {
            std::cout << "sent message: " << sendBuffer << " to device OK\n";
        }

        buffer_t inboundMessage = {
                .bytes = receiveBuffer,
                .size = sizeof(receiveBuffer)
        };

        if (i2c_recv(&ada_i2c_context, &inboundMessage)) {
            std::cout << "received message: " << receiveBuffer << " from device\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds (100));
    }

    std::chrono::time_point<std::chrono::steady_clock> end = std::chrono::steady_clock::now();

    std::cout << "took " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms" << std::endl;

    i2c_dev_close(&ada_i2c_context);

    return 0;
}
