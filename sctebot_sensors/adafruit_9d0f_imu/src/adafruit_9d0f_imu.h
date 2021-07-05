//
// Created by user on 4/9/21.
//

#ifndef ADAFRUIT_9D0F_IMU_ADAFRUIT_9D0F_IMU_H
#define ADAFRUIT_9D0F_IMU_ADAFRUIT_9D0F_IMU_H

#define ENABLE_BMP180_DEVICE 1
#define ENABLE_L3GD20_DEVICE 1
#define ENABLE_LSM303DLHC_DEVICE 1

#define ENABLE_MOCK_BMP180_DEVICE 1
#define ENABLE_MOCK_L3GD20_DEVICE 1
#define ENABLE_MOCK_LSM303DLHC_DEVICE 1

#include <iostream>
#include <thread>
#include <utility>
#include <condition_variable>

#include "common/i2c/i2c_linux.h"

#include "common/devices/bmp180.h"
#include "common/devices/l3gd20.h"
#include "common/devices/lsm303dlhc.h"

class AdaFruit9DoFImu {

private:
    int close_device() {

        return 0;
    }

public:
    AdaFruit9DoFImu() = default;

    ~AdaFruit9DoFImu() {
        std::cout << "destructor called" << std::endl;

        this->close_device();
    }

    int init_device() {

        return 1;
    }

};

#endif //ADAFRUIT_9D0F_IMU_ADAFRUIT_9D0F_IMU_H
