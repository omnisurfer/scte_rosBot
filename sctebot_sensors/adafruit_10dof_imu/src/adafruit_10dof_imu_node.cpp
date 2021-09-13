//
// Created by user on 9/12/21.
//
#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <cstring>

#include "adafruit_10dof_imu.h"

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

    std::cout << "Hello World adafruit 10dof IMU Node 2253" << std::endl;

    int i2c_bus_number = 0;
    //int i2c_device_address = 0x77;

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

    AdaFruit10DoFImu adaFruit10DoFImu = AdaFruit10DoFImu();

    adaFruit10DoFImu.init_device(
            i2c_bus_number,
            handle_bmp180_measurements,
            handle_l3gd20_measurements,
            handle_lsm303dlhc_accel_measurements,
            handle_lsm303dlhc_mag_measurements
    );

    adaFruit10DoFImu.run();

    std::cout << "press any key to exit..." << std::endl;

    std::cin.get();

    return 0;
}
