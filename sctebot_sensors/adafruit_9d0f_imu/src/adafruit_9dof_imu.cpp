//
// Created by user on 4/9/21.
//
#include <iostream>
#include <iomanip>
#include <string>
#include <math.h>
#include <cstring>

#include "adafruit_9dof_imu.h"

void handle_bmp180_measurements(float temperature, float pressure) {

    std::cout
        << "[bmp180]"
        << "\t\t temp (C): " << std::fixed << std::setprecision(2) << temperature
        << "\t\t pressure (Pa): " << std::fixed << std::setprecision(2) << pressure
        << std::endl;

    double absolute_altitude;
    double pressure_sea_level = 1013.25 * 100;

    double pressure_ratio = pressure / pressure_sea_level;
    double exponent_ratio = 1/5.255;

    absolute_altitude = 44330 * (1 - pow(pressure_ratio, exponent_ratio));

    double sea_level_pressure;

    sea_level_pressure = pressure / pow(1 - absolute_altitude/44330, 5.255);

    std::cout
        << "[bmp180]"
        << "\t\t abs alt (m): " << std::fixed << std::setprecision(2) << absolute_altitude
        << "\t\t sea (Pa): " << std::fixed << std::setprecision(2) << sea_level_pressure
        << std::endl;
}

void handle_l3gd20_measurements(float temperature, float r_x, float r_y, float r_z) {

    std::cout
        << "[l3gd20 gyro]"
        << "\t temp (C): " << std::fixed << std::setprecision(2) << temperature
        << "\t x_dps: " << std::fixed << std::setprecision(2) << r_x
        << "\t y_dps: " << std::fixed << std::setprecision(2) << r_y
        << "\t z_dps: " << std::fixed << std::setprecision(2) << r_z
        << std::endl;
}

void handle_lsm303dlhc_accel_measurements(float x_gs, float y_gs, float z_gs) {

    std::cout
            << "[lsm303 accel]"
            << "\t x_gs: " << std::fixed << std::setprecision(2) << x_gs
            << "\t y_gs: " << std::fixed << std::setprecision(2) << y_gs
            << "\t z_gs: " << std::fixed << std::setprecision(2) << z_gs
            << std::endl;
}

void handle_lsm303dlhc_mag_measurements(float temperature_deg_c, float x_ga, float y_ga, float z_ga) {

    std::cout
            << "[lsm303 mag]"
            << "\t temp (C): " << std::fixed << std::setprecision(2) << temperature_deg_c
            << "\t x_ga: " << std::fixed << std::setprecision(2) << x_ga
            << "\t y_ga: " << std::fixed << std::setprecision(2) << y_ga
            << "\t z_ga: " << std::fixed << std::setprecision(2) << z_ga
            << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "Hello World adafruit 10dof IMU" << std::endl;

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
    Bmp180Pressure bmp180DeviceHandle;

    /* CONFIRMED FOR BMP180 on RPI4 Node */
    i2c_device_address = BMP180_RPI_ADDRESS;

    bmp180DeviceHandle.config_device(
            i2c_bus_number,
            i2c_device_address,
            3000,
            "pressure_sensor",
            &handle_bmp180_measurements
    );

#if ENABLE_MOCK_BMP180_DEVICE
    bmp180DeviceHandle.enable_load_mock_data();
#endif

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
    L3gd20Gyro l3gd20GyroDeviceHandle;

    i2c_device_address = L3GD20_RPI_ADDRESS;

    l3gd20GyroDeviceHandle.config_device(
            i2c_bus_number,
            i2c_device_address,
            "3d_gyro",
            &handle_l3gd20_measurements
    );

#if ENABLE_MOCK_L3GD20_DEVICE
    l3gd20GyroDeviceHandle.enable_load_mock_data();
#endif

    if(!l3gd20GyroDeviceHandle.connect_to_device()) {
        return 0;
    };

#if ENABLE_MOCK_L3GD20_DEVICE
    l3gd20GyroDeviceHandle.mock_run_device_emulation();
#endif

    l3gd20GyroDeviceHandle.init_device(
            L3gd20Gyro::OutputDataRates::ODR_100P0HZ,
            L3gd20Gyro::BandwidthCutOff::MIN_CUT_OFF
            );
    //endregion
#endif

#if ENABLE_LSM303DLHC_ACCEL_DEVICE
    //region LSM303DLHC Accel device setup
    Lsm303DlhcAccelerometer lsm303DlhcAccelDeviceHandle;

    //LSM303DLHC may have two addresses, 0x19 for Accel, 0x1e for Mag/Temp

    i2c_device_address = LSM303DLHC_ACCEL_RPI_ADDRESS;

    lsm303DlhcAccelDeviceHandle.config_device(
            i2c_bus_number,
            i2c_device_address,
            "3d_accel",
            &handle_lsm303dlhc_accel_measurements
            );

    #if ENABLE_MOCK_LSM303DLHC_ACCEL_DEVICE
        lsm303DlhcAccelDeviceHandle.enable_load_mock_data();
    #endif

    if(!lsm303DlhcAccelDeviceHandle.connect_to_device()) {
        return 0;
    }

#if ENABLE_MOCK_LSM303DLHC_ACCEL_DEVICE
    lsm303DlhcAccelDeviceHandle.mock_run_device_emulation();
#endif

    lsm303DlhcAccelDeviceHandle.init_device(
            Lsm303DlhcAccelerometer::OutputDataRates::ODR_100P0HZ,
            Lsm303DlhcAccelerometer::HighPassFilterCutoff::MIN_CUT_OFF
            );
    //endregion
#endif

#if ENABLE_LSM303DLHC_MAG_DEVICE
    //region LSM303DLHC Mag device setup
    Lsm303DlhcMagnetometer lsm303DlhcMagDeviceHandle;

    i2c_device_address = LSM303DLHC_MAG_RPI_ADDRESS;

    lsm303DlhcMagDeviceHandle.config_device(
            i2c_bus_number,
            i2c_device_address,
            "3d_mag",
            &handle_lsm303dlhc_mag_measurements
            );

    #if ENABLE_MOCK_LSM303DLHC_MAG_DEVICE
    lsm303DlhcMagDeviceHandle.enable_load_mock_data();
    #endif

    if(!lsm303DlhcMagDeviceHandle.connect_to_device()) {
        return 0;
    }

    #if ENABLE_MOCK_LSM303DLHC_MAG_DEVICE
        lsm303DlhcMagDeviceHandle.mock_run_device_emulation();
    #endif

    lsm303DlhcMagDeviceHandle.init_device(Lsm303DlhcMagnetometer::ODR_75P0_HZ);
    //endregion
#endif

    std::cout << "press any key to continue..." << std::endl;

    std::cin.get();

    return 0;
}