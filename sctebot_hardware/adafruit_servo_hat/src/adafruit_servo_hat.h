//
// Created by user on 2/13/22.
//

#ifndef ADAFRUIT_SERVO_HAT_NODE_ADAFRUIT_SERVO_HAT_H
#define ADAFRUIT_SERVO_HAT_NODE_ADAFRUIT_SERVO_HAT_H

#define ENABLE_PCA9685_LED_DEVICE 1

#define PCA9685_RPI_ADDRESS 0x40

#include <iostream>
#include <thread>
#include <utility>
#include <condition_variable>

#include <geometry_msgs/TwistStamped.h>


#include "pca9685.h"

class AdaFruitServoHat {

private:

    std::unique_ptr<Pca9685LEDController> pca9685DeviceHandle;

    int close_device() {
        return 0;
    }

public:

    AdaFruitServoHat();

    ~AdaFruitServoHat() {

    }

    int init_device(
            int i2c_bus_number,
            void (*handle_pca9685_status)(int x, int y)
            ) {

        bool init_ok = true;

        pca9685DeviceHandle.reset(new Pca9685LEDController());

        int _i2c_bus_number = i2c_bus_number;
        int i2c_device_address;

#if ENABLE_PCA9685_LED_DEVICE
        i2c_device_address = PCA9685_RPI_ADDRESS;

        pca9685DeviceHandle->config_device(
                _i2c_bus_number,
                i2c_device_address,
                10,
                "pca9685_led_pwm",
                handle_pca9685_status
                );

        std::cout << "connecting to " << _i2c_bus_number << " at " << i2c_device_address << std::endl;

        if(!pca9685DeviceHandle->connect_to_device()) {
            init_ok = false;
        }
#endif

        return init_ok;
    }

    void run() {

#if ENABLE_PCA9685_LED_DEVICE

        int op_pwm_max_count_cycle = 4095;
        float op_pwm_on_percent = 0.0;
        float op_pwm_min_limit_duty_cycle = 0.03;
        float op_pwm_max_limit_duty_cycle = 0.125;
        float op_pwm_min_operating_duty_cycle = 0.03;
        float op_pwm_max_operating_duty_cycle = 0.125;
        float op_pwm_on_delay = 0.0;

        pca9685DeviceHandle->init_device(
                op_pwm_max_count_cycle,
                op_pwm_on_delay,
                op_pwm_min_limit_duty_cycle,
                op_pwm_max_limit_duty_cycle,
                op_pwm_min_operating_duty_cycle,
                op_pwm_max_operating_duty_cycle
                );
#endif
    }

    void command_pwm(Pca9685LEDController::LEDn led_n, float pwm_on_percent) {

        pca9685DeviceHandle->set_pwm(led_n, pwm_on_percent);

    }
};

#endif //ADAFRUIT_SERVO_HAT_NODE_ADAFRUIT_SERVO_HAT_H
