//
// Created by user on 4/9/21.
//
#include <iostream>
#include <string>
#include <math.h>
#include <cstring>

#include "adafruit_9dof_imu.h"

void handle_bmp180_measurements(float temperature, float pressure) {
    std::cout << "bmp180\t\ttemperature (C): " << temperature << " pressure (Pa): " << pressure << std::endl;
    /**/
    double absolute_altitude;
    double pressure_sea_level = 1013.25 * 100;

    double pressure_ratio = pressure / pressure_sea_level;
    double exponent_ratio = 1/5.255;

    absolute_altitude = 44330 * (1 - pow(pressure_ratio, exponent_ratio));

    double sea_level_pressure;

    sea_level_pressure = pressure / pow(1 - absolute_altitude/44330, 5.255);

    std::cout << "bmp180\t\tabs alt (m): " << absolute_altitude << " sea level pressure (Pa): " << sea_level_pressure << std::endl;
     /**/
}

void handle_l3gd20_measurements(int temperature, int r_x, int r_y, int r_z) {
    std::cout << "l3gd20\t\ttemp: " << (float)temperature << " x_dps: " << (float)r_x << " y_dps: " << (float)r_y << " z_dps: " << (float)r_z << std::endl;
}

void handle_lsm303dlhc_measurements(int temperature, int a_x, int a_y, int a_z, int m_x, int m_y, int m_z) {
    std::cout << "lsm303\t\ttemp: " << (float)temperature
        << " a_x: " << (float)a_x << " a_y: " << (float)a_y << " a_z: " << (float)a_z
        << " m_x: " << (float)m_x << " m_y: " << (float)m_y << " m_z: " << (float)m_z
        << std::endl;
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

#if ENABLE_BMP180_DEVICE
    //region BMP180 device setup
    Bmp180 bmp180DeviceHandle;

    bmp180DeviceHandle.config_device(
            i2c_bus_number,
            i2c_device_address,
            3000,
            "pressure_sensor",
            &handle_bmp180_measurements
    );

    if(!bmp180DeviceHandle.connect_to_device()) {
        return 0;
    };

#if ENABLE_MOCK_BMP180_DEVICE
    bmp180DeviceHandle.mock_run_device_emulation();
#endif

    bmp180DeviceHandle.init_device();
    //endregion
#endif

#if ENABLE_L3GD20_DEVICE
    //region L3GD20 device setup
    L3gd20 l3gd20DeviceHandle;

    i2c_device_address = 0x19;

    l3gd20DeviceHandle.config_device(
            i2c_bus_number,
            i2c_device_address,
            1000,
            "3d_gyro",
            &handle_l3gd20_measurements
    );

    if(!l3gd20DeviceHandle.connect_to_device()) {
        return 0;
    };

#if ENABLE_MOCK_L3GD20_DEVICE
    l3gd20DeviceHandle.mock_run_device_emulation();
#endif

    l3gd20DeviceHandle.init_device();
    //endregion
#endif

#if ENABLE_LSM303DLHC_DEVICE
    //region LSM303DLHC device setup
    Lsm303Dlhc lsm303DlhcDeviceHandle;

    i2c_device_address = 0x6b;

    lsm303DlhcDeviceHandle.config_device(
            i2c_bus_number,
            i2c_device_address,
            1000,
            "3d_accel",
            &handle_lsm303dlhc_measurements
    );

    if(!lsm303DlhcDeviceHandle.connect_to_device()) {
        return 0;
    }

#if ENABLE_MOCK_LSM303DLHC_DEVICE
    lsm303DlhcDeviceHandle.mock_run_device_emulation();
#endif

    lsm303DlhcDeviceHandle.init_device();
    //endregion
#endif

    std::cout << "press any key to continue..." << std::endl;

    std::cin.get();

    return 0;
}