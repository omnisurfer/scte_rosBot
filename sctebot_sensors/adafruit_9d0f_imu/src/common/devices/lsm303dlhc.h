//
// Created by user on 4/9/21.
//

#ifndef LSM303DLHC_H
#define LSM303DLHC_H

class Lsm303Dlhc {

private:

    int _lsm303dlhc_i2c_bus_number{};
    int _lsm303dlhc_i2c_device_address{};
    std::string _lsm303dlhc_device_name;

    context_t _lsm303dlhc_i2c_context{};

    int _init_lsm303dlhc() {

        return 0;
    }

    int _connect_to_lsm303dlhc() {
        return 0;
    }

public:

    int load_mock_lsm303dlhc_data() {
        return 0;
    }

};

#endif //LSM303DLHC_H
