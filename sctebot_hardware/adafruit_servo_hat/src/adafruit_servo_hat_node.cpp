//
// Created by user on 12/23/21.
//

#include "adafruit_servo_hat_node.h"

int main(int argc, char* argv[]) {

    std::unique_ptr<Pca9685LEDController> pca9685LedController(new Pca9685LEDController());

    pca9685LedController->init_device();

    std::cout << "Adafruit Servo Hat node running..." << std::endl;

}