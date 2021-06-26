//
// Created by user on 6/21/21.
//

#include <bitset>
#include "lsm303dlhc.h"

int Lsm303Dlhc::_init_device() {

    logging::core::get()->set_filter
    (
            logging::trivial::severity >= logging::trivial::debug
    );

    buffer_t inbound_message;
    buffer_t outbound_message;
    uint8_t register_address;
    uint8_t control_reg;

    // Read config bytes
    inbound_message = {
            .bytes = _control_register_buffer,
            .size = sizeof(_control_register_buffer)
    };

    register_address = Lsm303Dlhc::Addresses::CTRL_REG1_A;
    if(i2c_recv(&_i2c_device_context, &inbound_message, register_address)) {
        std::cout << "lsm303dlhc config: " << std::endl;

        for(uint i = 0; i < sizeof(_control_register_buffer); ++i) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)_control_register_buffer[i] << " ";
        }

        std::cout << std::endl;
    }

    // CONTROL REGISTERS

    // CTRL_REG1 - 1HZ
    control_reg =
            Lsm303Dlhc::BitMasks::ControlRegister1::ODR_1HZ |
            Lsm303Dlhc::BitMasks::Z_AXIS_EN |
            Lsm303Dlhc::BitMasks::Y_AXIS_EN |
            Lsm303Dlhc::BitMasks::X_AXIS_EN;

    std::cout << "ctrl1b "
                << std::bitset<8>(_control_register_buffer[0])
                << " ctrl1a " << std::bitset<8>(control_reg) << std::endl;

    register_address = Lsm303Dlhc::Addresses::CTRL_REG1_A;
    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(info) << "Reg 1A configure ok";
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "Reg 1A configure failed";
    }

    // CTRL_REG2 - DEFAULT
    // HPF @ 0.02Hz

    // CTRL_REG3 - DEFAULT

    // CTRL_REG4 - BDU = 1
    // LSB @ Lower address, LITTLE ENDIAN
    // FS Res +/- 2G
    // HiRes disabled
    control_reg =
            _control_register_buffer[3] |
            Lsm303Dlhc::BitMasks::ControlRegister4::BDU_EN;

    std::cout << "ctrl1a "
                << std::bitset<8>(_control_register_buffer[0])
                << " ctrl1b " << std::bitset<8>(control_reg) << std::endl;

    register_address = Lsm303Dlhc::Addresses::CTRL_REG4_A;
    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(info) << "Reg 4A configure ok";
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "Reg 4A configure failed";
    }

    // CTRL_REG5 - DEFAULT

    // CTRL_REG6 - DEFAULT

    // MAGNETIC REGISTERS

    // CRA_REG_M - Enable temperature
    // Data Output rate = 15Hz (default)
    inbound_message = {
            .bytes = _cra_reg_m,
            .size = sizeof(_cra_reg_m)
    };

    register_address = Lsm303Dlhc::Addresses::CRA_REG_M;
    if(i2c_recv(&_i2c_device_context, &inbound_message, register_address)) {
        std::cout << "lsm303dlhc config: " << std::endl;

        for(uint i = 0; i < sizeof(_cra_reg_m); ++i) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)_cra_reg_m[i] << " ";
        }

        std::cout << std::endl;
    }

    control_reg =
            _cra_reg_m[0] |
            Lsm303Dlhc::BitMasks::CrARegM::TEMP_EN;

    std::cout << "ctrlb mag "
            << std::bitset<8>(_control_register_buffer[0])
            << " ctrla mag " << std::bitset<8>(control_reg) << std::endl;

    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(info) << "Reg CRA_M configure ok";
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "Reg CRA_M configure failed";
    }

    // CRB_REG_M
    // Default gain (TBD)

    // MR_REG_M
    // Set to single-conversion mode
    inbound_message = {
            .bytes = _mr_reg_m,
            .size = sizeof(_mr_reg_m)
    };

    register_address = Lsm303Dlhc::Addresses::MR_REG_M;
    if(i2c_recv(&_i2c_device_context, &inbound_message, register_address)) {
        std::cout << "lsm303dlhc config: " << std::endl;

        for(uint i = 0; i < sizeof(_mr_reg_m); ++i) {
            std::cout << std::hex << std::setfill('0') << std::setw(2) << (int)_mr_reg_m[i] << " ";
        }

        std::cout << std::endl;
    }

    control_reg =
            _mr_reg_m[0] |
            Lsm303Dlhc::BitMasks::MrRegM::SINGLE_CONVERSION;

    if(i2c_send(&_i2c_device_context, &outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(info) << "Reg MR_REG_M configure ok";
    }
    else {
        BOOST_LOG_TRIVIAL(error) << "Reg MR_REG_M configure failed";
    }

    std::lock_guard<std::mutex> lk(this->data_capture_thread_run_mutex);
    this->data_capture_thread_run_cv.notify_one();
    this->run_data_capture_thread = true;

    return 0;
}

void Lsm303Dlhc::_data_capture_worker() {

}

void Lsm303Dlhc::_mock_device_emulation() {

}
