//
// Created by user on 6/26/21.
//

#ifndef ADAFRUIT_9D0F_IMU_SHARED_UTIL_H
#define ADAFRUIT_9D0F_IMU_SHARED_UTIL_H

#include <bitset>

void display_register_8bits(uint8_t reg_a, uint8_t reg_b) {

    /*
    std::cout << "reg a: " << std::bitset<8>(reg_a)<< "\rreg b: " << std::bitset<8>(reg_b) << std::endl;
    */
}

void display_register_16bits(uint16_t reg_a, uint16_t reg_b) {

    /*
    std::cout << "reg a: " << std::bitset<16>(reg_a)<< "\rreg b: " << std::bitset<16>(reg_b) << std::endl;
    */
}

#endif //ADAFRUIT_9D0F_IMU_SHARED_UTIL_H
