//
// Created by user on 4/9/21.
//

#ifndef L3GD20_H
#define L3GD20_H

class L3gd20 {

    // 1101 0101 0xD5 (Read address)
    // 1101 0100 0xD4 (Write address)

private:

    int _l3gd20_i2c_bus_number{};
    int _l3gd20_i2c_device_address{};
    std::string _l3gd20_device_name;

    context_t _l3gd20_i2c_context{};

    int _init_l3gd20() {

        return 0;
    }

    int _connect_to_l3gd20() {
        return 0;
    }

public:

    class Addresses {

    public:
        typedef enum Registers_t {
            WHO_AM_I = 0x0F,
            CTRL_REG1 = 0x20,
            CTRL_REG2 = 0x21,
            CTRL_REG3 = 0x22,
            CTRL_REG4 = 0x23,
            CTRL_REG5 = 0x24,
            REFERENCE = 0x25,
            OUT_TEMP = 0x26,
            STATUS_REG = 0x27,
            OUT_X_L = 0x28,
            OUT_X_H = 0x29,
            OUT_Y_L = 0x2A,
            OUT_Y_H = 0x2B,
            OUT_Z_L = 0x2C,
            OUT_Z_H = 0x2D,
            FIFO_CTRL_REG = 0x2E,
            FIFO_SRC_REG = 0x2F,
            INT1_CFG = 0x30,
            INT1_SRC = 0x31,
            INT1_TSH_XH = 0x32,
            INT1_TSH_XL = 0x33,
            INT1_TSH_YH = 0x34,
            INT1_TSH_YL = 0x35,
            INT1_TSH_ZH = 0x36,
            INT1_TSH_ZL = 0x37,
            INT1_DURATION = 0x38
        } Registers;
    };

    int load_mock_l3gd20_data() {
        return 0;
    }

};

#endif //L3GD20_H
