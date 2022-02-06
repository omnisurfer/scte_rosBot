//
// Created by user on 12/23/21.
//

#include "adafruit_servo_hat_node.h"

int main(int argc, char* argv[]) {

    std::unique_ptr<Pca9685LEDController> pca9685LedController(new Pca9685LEDController());

    int op_pwm_max_count_cycle = 4095;
    float op_pwm_on_percent = 0.0;
    float op_pwm_min_limit_duty_cycle = 0.03;
    float op_pwm_max_limit_duty_cycle = 0.125;
    float op_pwm_min_operating_duty_cycle = 0.03;
    float op_pwm_max_operating_duty_cycle = 0.125;
    float op_pwm_on_delay = 0.0;

    pca9685LedController->init_device(
            op_pwm_max_count_cycle,
            op_pwm_on_delay,
            op_pwm_min_limit_duty_cycle,
            op_pwm_max_limit_duty_cycle,
            op_pwm_min_operating_duty_cycle,
            op_pwm_max_operating_duty_cycle
            );

    std::cout << "Adafruit Servo Hat node running..." << std::endl;

}