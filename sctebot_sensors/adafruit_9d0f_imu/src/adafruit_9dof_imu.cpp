//
// Created by user on 4/9/21.
//
#include <iostream>
#include <string>
#include <math.h>
#include <cstring>

#include "adafruit_9d0f_imu.h"

void handle_bmp180_measurements(float temperature, float pressure) {
    std::cout << "temperature (C): " << temperature << " pressure (Pa): " << pressure << std::endl;
    /**/
    double absolute_altitude;
    double pressure_sea_level = 1013.25 * 100;

    double pressure_ratio = pressure / pressure_sea_level;
    double exponent_ratio = 1/5.255;

    absolute_altitude = 44330 * (1 - pow(pressure_ratio, exponent_ratio));

    double sea_level_pressure;

    sea_level_pressure = pressure / pow(1 - absolute_altitude/44330, 5.255);

    std::cout << "abs alt (m): " << absolute_altitude << " sea level pressure (Pa): " << sea_level_pressure << std::endl;
     /**/
}

void handle_l3gd20_measurements(float temperature, float pressure) {
    std::cout << "temperature (C): " << temperature << " pressure (Pa): " << pressure << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "Hello World adafruit 9dof IMU" << std::endl;

    int i2c_bus_number = 0;
    int i2c_device_address = 0x77;

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

    /* BMP180 device setup */
    Bmp180 bmp180DeviceHandle;

    bmp180DeviceHandle.config_bmp180(
            i2c_bus_number,
            i2c_device_address,
            1000,
            "pressure sensor",
            &handle_bmp180_measurements
    );

    if(!bmp180DeviceHandle.connect_to_device()) {
        return 0;
    };

    bmp180DeviceHandle.mock_load_bmp180_calibration_data();

#if ENABLE_MOCK_BMP180_DEVICE
    bmp180DeviceHandle.mock_run_bmp180_device_emulation();
#endif

    bmp180DeviceHandle.init_device();

    /* L3GD20 device setup */
    L3gd20 l3gd20DeviceHandle;

    i2c_device_address = 0x6b;

    l3gd20DeviceHandle.config_l3gd20(
            i2c_bus_number,
            i2c_device_address,
            1000,
            "3d_gyro",
            &handle_l3gd20_measurements
    );

    if(!l3gd20DeviceHandle.connect_to_device()) {
        return 0;
    };

    l3gd20DeviceHandle.load_mock_l3gd20_data();

    l3gd20DeviceHandle.init_device();

    std::cout << "press any key to continue..." << std::endl;

    std::cin.get();

    return 0;
}