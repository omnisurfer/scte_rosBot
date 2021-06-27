//
// Created by user on 6/26/21.
//

#ifndef ADAFRUIT_9D0F_IMU_SHARED_UTIL_H
#define ADAFRUIT_9D0F_IMU_SHARED_UTIL_H

#include <bitset>

void display_register_bits(uint8_t reg_a, uint8_t reg_b) {

    std::cout << "reg a: " << std::bitset<8>(reg_a)<< "\r\nreg b: " << std::bitset<8>(reg_b) << std::endl;

}

#endif //ADAFRUIT_9D0F_IMU_SHARED_UTIL_H
