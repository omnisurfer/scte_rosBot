//
// Created by user on 6/26/21.
//

#ifndef ADAFRUIT_9D0F_IMU_SHARED_UTIL_H
#define ADAFRUIT_9D0F_IMU_SHARED_UTIL_H

#define ENABLE_REGISTER_DISPLAY 1

#include <bitset>
#include <string>

void display_register_8bits(const std::string& reg_a_name, uint8_t reg_a, const std::string& reg_b_name, uint8_t reg_b) {

#if ENABLE_REGISTER_DISPLAY
    std::cout << reg_a_name << ": " << std::bitset<8>(reg_a) << " " << reg_a_name << ": " << std::bitset<8>(reg_b) << std::endl;
#endif

}

void display_register_16bits(const std::string& reg_a_name, uint16_t reg_a, const std::string& reg_b_name, uint16_t reg_b) {

#if ENABLE_REGISTER_DISPLAY
    std::cout << reg_a_name << ": " << std::bitset<16>(reg_a)<< " " << reg_b_name << ": " << std::bitset<16>(reg_b) << std::endl;
#endif

}

#endif //ADAFRUIT_9D0F_IMU_SHARED_UTIL_H
