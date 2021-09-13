//
// Created by user on 4/9/21.
//

#ifndef ADAFRUIT_9D0F_IMU_ADAFRUIT_9DOF_IMU_H
#define ADAFRUIT_9D0F_IMU_ADAFRUIT_9DOF_IMU_H

#define ENABLE_BMP180_DEVICE 1
#define ENABLE_L3GD20_DEVICE 1
#define ENABLE_LSM303DLHC_ACCEL_DEVICE 1
#define ENABLE_LSM303DLHC_MAG_DEVICE 1

#define ENABLE_MOCK_BMP180_DEVICE 1
#define ENABLE_MOCK_L3GD20_DEVICE 1
#define ENABLE_MOCK_LSM303DLHC_ACCEL_DEVICE 1
#define ENABLE_MOCK_LSM303DLHC_MAG_DEVICE 1

#define BMP180_RPI_ADDRESS 0x77
#define L3GD20_RPI_ADDRESS 0x6b
#define LSM303DLHC_ACCEL_RPI_ADDRESS 0x19
#define LSM303DLHC_MAG_RPI_ADDRESS 0x1e

#include <iostream>
#include <thread>
#include <utility>
#include <condition_variable>

#include "common/devices/bmp180.h"
#include "common/devices/l3gd20.h"
#include "common/devices/lsm303dlhc.h"


class AdaFruit9DoFImu {

private:

    // Declared here because of deleted copy constructor of mutex
    // so creating a pointer to the object
    // https://stackoverflow.com/questions/58386020/how-to-fix-use-of-deleted-function-when-using-mutex-and-condition-variable-as

    Bmp180Pressure *bmp180DeviceHandle;
    L3gd20Gyro *l3gd20GyroDeviceHandle;
    Lsm303DlhcAccelerometer *lsm303DlhcAccelDeviceHandle;
    Lsm303DlhcMagnetometer *lsm303DlhcMagDeviceHandle;

    int close_device() {

        return 0;
    }

public:

    AdaFruit9DoFImu();

    ~AdaFruit9DoFImu() {
        std::cout << "AdaFruit9DoFImu destructor called" << std::endl;

        this->close_device();
    }

    int init_device(
            int i2c_bus_number,
            void (*handle_bmp180_measurements)(float temperature, float pressure),
            void (*handle_l3gd20_measurements)(float temperature, float r_x, float r_y, float r_z),
            void (*handle_lsm303dlhc_accel_measurements)(float x_gs, float y_gs, float z_gs),
            void (*handle_lsm303dlhc_mag_measurements)(float temperature_deg_c, float x_ga, float y_ga, float z_ga)
            ) {

        int _i2c_bus_number = i2c_bus_number;
        int i2c_device_address;

#if ENABLE_BMP180_DEVICE
        /* CONFIRMED FOR BMP180 on RPI4 Node */
        i2c_device_address = BMP180_RPI_ADDRESS;

        bmp180DeviceHandle->config_device(
                _i2c_bus_number,
                i2c_device_address,
                3000,
                "pressure_sensor",
                handle_bmp180_measurements
        );

#if ENABLE_MOCK_BMP180_DEVICE
        bmp180DeviceHandle->enable_load_mock_data();
#endif
        if(!bmp180DeviceHandle->connect_to_device()) {
            return -1;
        };
#endif

#if ENABLE_L3GD20_DEVICE
        i2c_device_address = L3GD20_RPI_ADDRESS;

        l3gd20GyroDeviceHandle->config_device(
                _i2c_bus_number,
                i2c_device_address,
                "3d_gyro",
                handle_l3gd20_measurements
        );

#if ENABLE_MOCK_L3GD20_DEVICE
        l3gd20GyroDeviceHandle->enable_load_mock_data();
#endif
        if(!l3gd20GyroDeviceHandle->connect_to_device()) {
            return -1;
        };
#endif

#if ENABLE_LSM303DLHC_ACCEL_DEVICE
        //LSM303DLHC may have two addresses, 0x19 for Accel, 0x1e for Mag/Temp
        i2c_device_address = LSM303DLHC_ACCEL_RPI_ADDRESS;

        lsm303DlhcAccelDeviceHandle->config_device(
                _i2c_bus_number,
                i2c_device_address,
                "3d_accel",
                handle_lsm303dlhc_accel_measurements
        );

#if ENABLE_MOCK_LSM303DLHC_ACCEL_DEVICE
        lsm303DlhcAccelDeviceHandle->enable_load_mock_data();
#endif
        if(!lsm303DlhcAccelDeviceHandle->connect_to_device()) {
            return -1;
        }
#endif

#if ENABLE_LSM303DLHC_MAG_DEVICE
        i2c_device_address = LSM303DLHC_MAG_RPI_ADDRESS;

        lsm303DlhcMagDeviceHandle->config_device(
                _i2c_bus_number,
                i2c_device_address,
                "3d_mag",
                handle_lsm303dlhc_mag_measurements
        );

#if ENABLE_MOCK_LSM303DLHC_MAG_DEVICE
        lsm303DlhcMagDeviceHandle->enable_load_mock_data();
#endif
        if(!lsm303DlhcMagDeviceHandle->connect_to_device()) {
            return -1;
        }
#endif
        return 1;
    }

    void run() {

#if ENABLE_BMP180_DEVICE
#if ENABLE_MOCK_BMP180_DEVICE
        bmp180DeviceHandle->mock_run_device_emulation();
#endif
        bmp180DeviceHandle->init_device();
#endif

#if ENABLE_L3GD20_DEVICE
#if ENABLE_MOCK_L3GD20_DEVICE
        l3gd20GyroDeviceHandle->mock_run_device_emulation();
#endif
        l3gd20GyroDeviceHandle->init_device(
                L3gd20Gyro::OutputDataRates::ODR_12P5HZ,
                L3gd20Gyro::BandwidthCutOff::MIN_CUT_OFF
        );
#endif

#if ENABLE_LSM303DLHC_ACCEL_DEVICE
#if ENABLE_MOCK_LSM303DLHC_ACCEL_DEVICE
        lsm303DlhcAccelDeviceHandle->mock_run_device_emulation();
#endif
        lsm303DlhcAccelDeviceHandle->init_device(
                Lsm303DlhcAccelerometer::OutputDataRates::ODR_10P0HZ,
                Lsm303DlhcAccelerometer::HighPassFilterCutoff::MIN_CUT_OFF,
                Lsm303DlhcAccelerometer::SensorAccelerationFullScale::PN_2G
        );
#endif

#if ENABLE_LSM303DLHC_MAG_DEVICE
#if ENABLE_MOCK_LSM303DLHC_MAG_DEVICE
        lsm303DlhcMagDeviceHandle->mock_run_device_emulation();
#endif
        lsm303DlhcMagDeviceHandle->init_device(
                Lsm303DlhcMagnetometer::OutputDataRates::ODR_7P5_HZ,
                Lsm303DlhcMagnetometer::SensorMagnetometerFullScale::PN_1P3G
        );
#endif
    }
};

#endif //ADAFRUIT_9D0F_IMU_ADAFRUIT_9DOF_IMU_H
