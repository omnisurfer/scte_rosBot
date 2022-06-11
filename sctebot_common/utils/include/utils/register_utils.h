//
// Created by user on 6/26/21.
//

#ifndef REGISTER_UTILS_H
#define REGISTER_UTILS_H

#define ENABLE_REGISTER_DISPLAY 0

#include <bitset>
#include <string>
#include <sstream>

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

#endif //REGISTER_UTILS_H
