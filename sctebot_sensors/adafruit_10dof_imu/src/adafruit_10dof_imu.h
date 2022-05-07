//
// Created by user on 4/9/21.
//

#ifndef ADAFRUIT_10DOF_IMU_H
#define ADAFRUIT_10DOF_IMU_H

#define ENABLE_BMP180_PRESSURE_DEVICE 1
#define ENABLE_L3GD20_GYRO_DEVICE 1
#define ENABLE_LSM303DLHC_ACCEL_DEVICE 1
#define ENABLE_LSM303DLHC_MAG_DEVICE 1

#define ENABLE_MOCK_BMP180_DEVICE 0
#define ENABLE_MOCK_L3GD20_DEVICE 0
#define ENABLE_MOCK_LSM303DLHC_ACCEL_DEVICE 0
#define ENABLE_MOCK_LSM303DLHC_MAG_DEVICE 0

#define BMP180_RPI_ADDRESS 0x77
#define L3GD20_RPI_ADDRESS 0x6b
#define LSM303DLHC_ACCEL_RPI_ADDRESS 0x19
#define LSM303DLHC_MAG_RPI_ADDRESS 0x1e

#include <iostream>
#include <thread>
#include <utility>
#include <condition_variable>

#include "bmp180.h"
#include "l3gd20.h"
#include "lsm303dlhc.h"


class AdaFruit10DoFImu {

private:

    std::string _device_name = "adafruit_10dof_imu";

    // Declared here because of deleted copy constructor of mutex
    // so creating a pointer to the object
    // https://stackoverflow.com/questions/58386020/how-to-fix-use-of-deleted-function-when-using-mutex-and-condition-variable-as

    std::unique_ptr<Bmp180Pressure> bmp180DeviceHandle;
    std::unique_ptr<L3gd20Gyro> l3gd20GyroDeviceHandle;
    std::unique_ptr<Lsm303DlhcAccelerometer> lsm303DlhcAccelDeviceHandle;
    std::unique_ptr<Lsm303DlhcMagnetometer> lsm303DlhcMagDeviceHandle;

    std::mutex bmp180_data_capture_worker_execute_cycle_mutex;
    std::condition_variable bmp180_data_capture_worker_execute_cycle_cv;

    std::mutex l3gd20_data_capture_worker_execute_cycle_mutex;
    std::condition_variable l3gd20_data_capture_worker_execute_cycle_cv;

    std::mutex lsm303dlhc_accel_data_capture_worker_execute_cycle_mutex;
    std::condition_variable lsm303dlhc_accel_data_capture_worker_execute_cycle_conditional_variable;

    std::mutex lsm303dlhc_mag_data_capture_worker_execute_cycle_mutex;
    std::condition_variable lsm303dlhc_mag_data_capture_worker_execute_cycle_conditional_variable;

    bool run_data_capture_execute_management_thread = false;
    bool is_running_data_capture_execute_management_thread = false;
    std::mutex data_capture_execute_management_thread_run_mutex;
    std::condition_variable data_capture_execute_management_thread_run_cv;
    std::thread data_capture_execute_management_thread;


    int close_device() {

        bool data_capture_execute_management_thread_was_running = false;

        std::unique_lock<std::mutex> execute_management_lock(this->data_capture_execute_management_thread_run_mutex);
        {
            if (this->is_running_data_capture_execute_management_thread) {
                data_capture_execute_management_thread_was_running = this->is_running_data_capture_execute_management_thread;
                this->run_data_capture_execute_management_thread = false;
                this->data_capture_execute_management_thread_run_cv.notify_one();
            }
            execute_management_lock.unlock();
        }

        if(data_capture_execute_management_thread_was_running) {

            if(data_capture_execute_management_thread.joinable()) {
                data_capture_execute_management_thread.join();
            }
        }

        return 0;
    }

    void _data_capture_execute_management_worker();

public:

    AdaFruit10DoFImu();

    ~AdaFruit10DoFImu() {
        std::cout << "AdaFruit10DoFImu destructor called" << std::endl;

        this->close_device();
    }

    int init_device(
            int i2c_bus_number,
            void (*handle_bmp180_measurements)(float temperature, float pressure),
            void (*handle_l3gd20_measurements)(float temperature, float r_x, float r_y, float r_z),
            void (*handle_lsm303dlhc_accel_measurements)(float x_gs, float y_gs, float z_gs),
            void (*handle_lsm303dlhc_mag_measurements)(float temperature_deg_c, float x_ga, float y_ga, float z_ga)
            ) {

        ScteBotBoostLogger sctebot_boost_logger = ScteBotBoostLogger();
        sctebot_boost_logger.init_boost_logging();

        BOOST_LOG_TRIVIAL(info) << "adafruit_10dof_imu init_device";

        bool init_ok = true;

        bmp180DeviceHandle.reset(
                new Bmp180Pressure(this->bmp180_data_capture_worker_execute_cycle_mutex,
                                   this->bmp180_data_capture_worker_execute_cycle_cv)
                );

        l3gd20GyroDeviceHandle.reset(
                new L3gd20Gyro(this->l3gd20_data_capture_worker_execute_cycle_mutex,
                               this->l3gd20_data_capture_worker_execute_cycle_cv)
                );

        lsm303DlhcAccelDeviceHandle.reset(
                new Lsm303DlhcAccelerometer(this->lsm303dlhc_accel_data_capture_worker_execute_cycle_mutex,
                                            this->lsm303dlhc_accel_data_capture_worker_execute_cycle_conditional_variable)
                );

        lsm303DlhcMagDeviceHandle.reset(
                new Lsm303DlhcMagnetometer(this->lsm303dlhc_mag_data_capture_worker_execute_cycle_mutex,
                                           this->lsm303dlhc_mag_data_capture_worker_execute_cycle_conditional_variable)
                );

        int _i2c_bus_number = i2c_bus_number;
        int _i2c_device_address;

#if ENABLE_BMP180_PRESSURE_DEVICE

        /* CONFIRMED FOR BMP180 on RPI4 Node */
        _i2c_device_address = BMP180_RPI_ADDRESS;

        bmp180DeviceHandle->config_device(
                _i2c_bus_number,
                _i2c_device_address,
                "bmp180_pressure",
                handle_bmp180_measurements
        );

        std::unique_lock<std::mutex> lock(bmp180_data_capture_worker_execute_cycle_mutex);
        bmp180_data_capture_worker_execute_cycle_cv.notify_all();
        lock.unlock();

        #if ENABLE_MOCK_BMP180_DEVICE
        bmp180DeviceHandle->enable_load_mock_data();
        #endif

        if(!bmp180DeviceHandle->connect_to_device()) {

            init_ok = false;
        };
#endif

#if ENABLE_L3GD20_GYRO_DEVICE

        _i2c_device_address = L3GD20_RPI_ADDRESS;

        l3gd20GyroDeviceHandle->config_device(
                _i2c_bus_number,
                _i2c_device_address,
                "l3gd20_gyro",
                handle_l3gd20_measurements
        );

        #if ENABLE_MOCK_L3GD20_DEVICE
        l3gd20GyroDeviceHandle->enable_load_mock_data();
        #endif

        if(!l3gd20GyroDeviceHandle->connect_to_device()) {

            init_ok = false;
        };
#endif

#if ENABLE_LSM303DLHC_ACCEL_DEVICE

        //LSM303DLHC may have two addresses, 0x19 for Accel, 0x1e for Mag/Temp
        _i2c_device_address = LSM303DLHC_ACCEL_RPI_ADDRESS;

        lsm303DlhcAccelDeviceHandle->config_device(
                _i2c_bus_number,
                _i2c_device_address,
                "lsm303dlhc_accel",
                handle_lsm303dlhc_accel_measurements
        );

        #if ENABLE_MOCK_LSM303DLHC_ACCEL_DEVICE
        lsm303DlhcAccelDeviceHandle->enable_load_mock_data();
        #endif

        if(!lsm303DlhcAccelDeviceHandle->connect_to_device()) {

            init_ok = false;
        }
#endif

#if ENABLE_LSM303DLHC_MAG_DEVICE

        _i2c_device_address = LSM303DLHC_MAG_RPI_ADDRESS;

        lsm303DlhcMagDeviceHandle->config_device(
                _i2c_bus_number,
                _i2c_device_address,
                "lsm303dlhc_mag",
                handle_lsm303dlhc_mag_measurements
        );

        #if ENABLE_MOCK_LSM303DLHC_MAG_DEVICE
        lsm303DlhcMagDeviceHandle->enable_load_mock_data();
        #endif

        if(!lsm303DlhcMagDeviceHandle->connect_to_device()) {

            init_ok = false;
        }
#endif

        if(init_ok) {
            this->data_capture_execute_management_thread = std::thread(&AdaFruit10DoFImu::_data_capture_execute_management_worker, this);
        }

        return init_ok;
    }

    void run() {

#if ENABLE_BMP180_PRESSURE_DEVICE

        #if ENABLE_MOCK_BMP180_DEVICE
        bmp180DeviceHandle->mock_run_device_emulation();
        #endif

        // just picked 0.33Hz rate
        bmp180DeviceHandle->init_device(300);
#endif

#if ENABLE_L3GD20_GYRO_DEVICE

        #if ENABLE_MOCK_L3GD20_DEVICE
        l3gd20GyroDeviceHandle->mock_run_device_emulation();
        #endif

        l3gd20GyroDeviceHandle->init_device(
                L3gd20Gyro::OutputDataRates::ODR_100P0HZ, //ODR_50P0HZ, //ODR_12P5HZ_BM,
                L3gd20Gyro::BandwidthCutOff::MIN_CUT_OFF
        );
#endif

#if ENABLE_LSM303DLHC_ACCEL_DEVICE

        #if ENABLE_MOCK_LSM303DLHC_ACCEL_DEVICE
        lsm303DlhcAccelDeviceHandle->mock_run_device_emulation();
        #endif

        lsm303DlhcAccelDeviceHandle->init_device(
                Lsm303DlhcAccelerometer::OutputDataRates::ODR_100P0HZ, //ODR_100P0HZ, //ODR_1P0HZ,
                Lsm303DlhcAccelerometer::HighPassFilterCutoff::MIN_CUT_OFF,
                Lsm303DlhcAccelerometer::SensorAccelerationFullScale::PN_2G
        );
#endif

#if ENABLE_LSM303DLHC_MAG_DEVICE

        #if ENABLE_MOCK_LSM303DLHC_MAG_DEVICE
        lsm303DlhcMagDeviceHandle->mock_run_device_emulation();
        #endif

        lsm303DlhcMagDeviceHandle->init_device(
                Lsm303DlhcMagnetometer::OutputDataRates::ODR_15P0_HZ, // ODR_75P0_HZ, //ODR_1P5_HZ,
                Lsm303DlhcMagnetometer::SensorMagnetometerFullScale::PN_1P3G
        );
#endif

        std::unique_lock<std::mutex> execute_management_lock(data_capture_execute_management_thread_run_mutex);
        this->run_data_capture_execute_management_thread = true;
        this->data_capture_execute_management_thread_run_cv.notify_all();
        execute_management_lock.unlock();
    }
};

#endif //ADAFRUIT_10DOF_IMU_H
