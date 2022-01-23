//
// Created by user on 12/17/21.
//

#ifndef PCA9685_H
#define PCA9685_H

#include <iostream>
#include <string>
#include <thread>
#include <condition_variable>

#include <chrono>
#include <thread>
#include <bitset>

#include <iostream>

#include "i2c_linux/i2c_linux.h"
#include "utils/boost_logging.h"

#define ENABLE 0xFF
#define DISABLE 0x00

class Pca9685LEDController {

public:

    typedef enum LEDn_t {
        LED0 = 0,
        LED1 = 1,
        LED2 = 2,
        LED3 = 3,
        LED4 = 4,
        LED5 = 5,
        LED6 = 6,
        LED7 = 7,

        LED8 = 8,
        LED9 = 9,
        LED10 = 10,
        LED11 = 11,
        LED12 = 12,
        LED13 = 13,
        LED14 = 14,
        LED15 = 15
    } LEDn;

private:

    class Addresses {

    public:
        typedef enum Registers_t {
            MODE1 = 0x00,
            MODE2 = 0x01,
            SUBADR1 = 0x02,
            SUBADR2 = 0x03,
            SUBADR3 = 0x04,
            ALLCALLADR = 0x05,

            LED0_ON_L = 0x06,
            LED0_ON_H = 0x07,
            LED0_OFF_L = 0x08,
            LED0_OFF_H = 0x09,

            LED1_ON_L = 0x0A,
            LED1_ON_H = 0x0B,
            LED1_OFF_L = 0x0C,
            LED1_OFF_H = 0x0D,

            LED2_ON_L = 0x0E,
            LED2_ON_H = 0x0F,
            LED2_OFF_L = 0x10,
            LED2_OFF_H = 0x11,

            LED3_ON_L = 0x12,
            LED3_ON_H = 0x13,
            LED3_OFF_L = 0x14,
            LED3_OFF_H = 0x15,

            LED4_ON_L = 0x16,
            LED4_ON_H = 0x17,
            LED4_OFF_L = 0x18,
            LED4_OFF_H = 0x19,

            LED5_ON_L = 0x1A,
            LED5_ON_H = 0x1B,
            LED5_OFF_L = 0x1C,
            LED5_OFF_H = 0x1D,

            LED6_ON_L = 0x1E,
            LED6_ON_H = 0x1F,
            LED6_OFF_L = 0x20,
            LED6_OFF_H = 0x21,

            LED7_ON_L = 0x22,
            LED7_ON_H = 0x23,
            LED7_OFF_L = 0x24,
            LED7_OFF_H = 0x25,

            LED8_ON_L = 0x26,
            LED8_ON_H = 0x27,
            LED8_OFF_L = 0x28,
            LED8_OFF_H = 0x29,

            LED9_ON_L = 0x2A,
            LED9_ON_H = 0x2B,
            LED9_OFF_L = 0x2C,
            LED9_OFF_H = 0x2D,

            LED10_ON_L = 0x2E,
            LED10_ON_H = 0x2F,
            LED10_OFF_L = 0x30,
            LED10_OFF_H = 0x31,

            LED11_ON_L = 0x32,
            LED11_ON_H = 0x33,
            LED11_OFF_L = 0x34,
            LED11_OFF_H = 0x35,

            LED12_ON_L = 0x36,
            LED12_ON_H = 0x37,
            LED12_OFF_L = 0x38,
            LED12_OFF_H = 0x39,

            LED13_ON_L = 0x3A,
            LED13_ON_H = 0x3B,
            LED13_OFF_L = 0x3C,
            LED13_OFF_H = 0x3D,

            LED14_ON_L = 0x3E,
            LED14_ON_H = 0x3F,
            LED14_OFF_L = 0x40,
            LED14_OFF_H = 0x41,

            LED15_ON_L = 0x42,
            LED15_ON_H = 0x43,
            LED15_OFF_L = 0x44,
            LED15_OFF_H = 0x45,

            ALL_LED_ON_L = 0xFA,
            ALL_LED_ON_H = 0xFB,
            ALL_LED_OFF_L = 0xFC,
            ALL_LED_OFF_H = 0xFD,

            PRE_SCALE = 0xFE,
            TEST_MODE = 0xFF

        } Registers;
    };

    class BitMasks {

    public:

        typedef enum Mode1_t {
            RESTART = (1 << 7),
            EXTCLK = (1 << 6),
            AI = (1 << 5),
            SLEEP = (1 << 4),
            SUB1 = (1 << 3),
            SUB2 = (1 << 2),
            SUB3 = (1 << 1),
            ALLCALL = (1 << 0)
        } Mode1;

        typedef enum Mode2_t {
            INVRT = (1 << 4),
            OCH = (1 << 3),
            OUTDRV = (1 << 2),
            OUTNE_LEDN_HIGH_Z = (1 << 1),
            OUTNE_LEDN_SET_1 = (1 << 0),
        } Mode2;
    };

    std::mutex _i2c_device_mutex;
    int _i2c_bus_number{};
    int _i2c_device_address{};
    int _sensor_update_period_ms{};
    std::string _device_name;

    context_t _i2c_device_context{};

    typedef void (*host_callback_function)(
            int x, int y
    );

    bool run_servo_status_thread = false;
    std::condition_variable run_servo_status_thread_cv;
    std::mutex run_servo_status_thread_mutex;
    std::thread servo_status_thread;

    host_callback_function _host_callback_function{};

    int _connect_to_device() {

        /*
         * setup the i2c context and connect
         */
        _i2c_device_context = {0};
        if(!i2c_dev_open(&_i2c_device_context, _i2c_bus_number, _i2c_device_address)) {
            //BOOST_LOG_TRIVIAL(error) << this->_device_name << ": failed to open device";
            return 0;
        }

        if(!i2c_is_connected(&_i2c_device_context)) {
            //BOOST_LOG_TRIVIAL(error) << this->_device_name << ": failed to connect to device";
            return 0;
        }

        /*
        // try read chip id
        uint8_t chip_id[1] = {0};
        buffer_t inbound_message = {
                .bytes = chip_id,
                .size = sizeof(chip_id)
        };

        uint8_t register_address;
        register_address = Bmp180Pressure::Addresses::DataRegisters::CHIP_ID;
        i2c_recv(&_i2c_device_context, &inbound_message, register_address);

        if(chip_id[0] != Bmp180Pressure::MagicNumbers::ChipId::CHIP_ID) {
            BOOST_LOG_TRIVIAL(error) << this->_device_name << ": failed to read device chip id";
            return 0;
        }
        */

        return 1;
    }

    int _init_device();

    int _close_device() {

        close_i2c_dev(_i2c_bus_number);

        return 0;
    }

    void _servo_management_worker();

    void _restart_device() {

        std::string device_name = this->_device_name;

        BOOST_LOG_TRIVIAL(debug) << device_name <<": _restart_device() enter" << std::endl;

        buffer_t inbound_message;
        buffer_t outbound_message;
        uint8_t register_address;
        uint8_t control_reg[1] = {0};

        int restart_ok = 1;

        register_address = Pca9685LEDController::Addresses::Registers::MODE1;

        // Read mode 1 reg
        uint8_t mode1_register_data[1];

        inbound_message = {
                .bytes = mode1_register_data,
                .size = sizeof(mode1_register_data)
        };

        if(receive_i2c(&inbound_message, register_address)) {
            // nothing to do here
        }
        else {
            restart_ok = 0;
        }

        bool clear_sleep = false;

        // Check bit 7 (RESTART) and clear bit 4 (SLEEP)
        if(mode1_register_data[0] & Pca9685LEDController::BitMasks::Mode1::RESTART) {
            BOOST_LOG_TRIVIAL(debug) << device_name << ": Restart asserted, waking device from sleep";
            clear_sleep = true;
        }
        else {
            restart_ok = 0;
        }

        if(clear_sleep) {
            control_reg[0] = mode1_register_data[0] & ~(Pca9685LEDController::BitMasks::Mode1::SLEEP);

            outbound_message = {
                    .bytes = control_reg,
                    .size = sizeof(control_reg)
            };

            if(send_i2c(&outbound_message, register_address)) {
                BOOST_LOG_TRIVIAL(debug) << device_name <<": MODE1 register configure OK (b" << std::bitset<8>(control_reg[0]) << ")";
            }
            else {
                BOOST_LOG_TRIVIAL(debug) << device_name <<": MODE1 register configuration failed (b" << std::bitset<8>(control_reg[0]) << ")";

                restart_ok = 0;
            }
        }

        // wait 500us
        std::this_thread::sleep_for(std::chrono::microseconds (500));

        // Write 1 to bit 7 of MODE1
#if 0
        control_reg[0] = mode1_register_data[0] & ~(Pca9685LEDController::BitMasks::Mode1::RESTART);

        outbound_message = {
                .bytes = control_reg,
                .size = sizeof(control_reg)
        };

        if(send_i2c(&outbound_message, register_address)) {
            BOOST_LOG_TRIVIAL(debug) << device_name <<": MODE1 register configure OK (b" << std::bitset<8>(control_reg[0]) << ")";
        }
        else {
            BOOST_LOG_TRIVIAL(debug) << device_name <<": MODE1 register configuration failed (b" << std::bitset<8>(control_reg[0]) << ")";

            restart_ok = 0;
        }
#endif
        BOOST_LOG_TRIVIAL(debug) << device_name <<": _restart_device() exit" << std::endl;
    }

    void _set_pwm(LEDn led_n, uint16_t pwm_on, uint16_t pwm_off) {

        if(pwm_on > 4095) {
            pwm_on = 4095;
        }

        if(pwm_off > 4095) {
            pwm_off = 4095;
        }

        // per datasheet make sure we don't have the same value in both registers
        if(pwm_off == pwm_on) {
            pwm_off = pwm_off + 1;
        }

        std::cout << "setting led #" << led_n << " pwm_on " << pwm_on << " pwm_off " << pwm_off << std::endl;

        int setting_ok = 1;

        buffer_t inbound_message;
        buffer_t outbound_message;
        uint8_t register_address;
        uint8_t ledn_on_low[1] = {0};
        uint8_t ledn_on_high[1] = {0};
        uint8_t ledn_off_low[1] = {0};
        uint8_t ledn_off_high[1] = {0};

        ledn_on_low[0] = (pwm_on & 0x00FF);
        ledn_on_high[0] = (pwm_on & 0xFF00) >> 8;

        ledn_off_low[0] = (pwm_off & 0x00F);
        ledn_off_high[0] = (pwm_off & 0xFF00) >> 8;

        register_address = Pca9685LEDController::Addresses::LED0_ON_L;

        uint8_t led_register_data[4];
        led_register_data[0] = ledn_on_low[0];
        led_register_data[1] = ledn_on_high[0];
        led_register_data[2] = ledn_off_low[0];
        led_register_data[3] = ledn_off_high[0];

        outbound_message = {
                .bytes = led_register_data,
                .size = sizeof(led_register_data)
        };

        if(send_i2c(&outbound_message, register_address)) {
            // do nothing
        }
        else {
            // do nothing
        }
    }

    int receive_i2c(buffer_t *data, uint8_t register_address);
    int send_i2c(buffer_t *data, uint8_t register_address);
    int open_i2c_dev(int device_number, int slave_address);
    int is_i2c_dev_connected();
    void close_i2c_dev(int bus_number);

public:

    Pca9685LEDController() = default;

    ~Pca9685LEDController() {

        std::unique_lock<std::mutex> run_lock(this->run_servo_status_thread_mutex);
        this->run_servo_status_thread = false;
        this->run_servo_status_thread_cv.notify_all();
        run_lock.unlock();

        if(servo_status_thread.joinable()) {
            servo_status_thread.join();
        }

    }

    int config_device(
            int bus_number,
            int device_address,
            int update_period_ms,
            std::string device_name,
            host_callback_function
            function_pointer
    ) {
        _i2c_bus_number = bus_number;
        _i2c_device_address = device_address;
        _sensor_update_period_ms = update_period_ms;
        _device_name = std::move(device_name);

        _host_callback_function = function_pointer;

        servo_status_thread = std::thread(&Pca9685LEDController::_servo_management_worker, this);

        return 0;
    }

    int connect_to_device() {
        int status = 1;

        status &= this->_connect_to_device();

        return status;
    }

    int init_device() {

        return  this->_init_device();

    }

    void set_pwm(LEDn led_n, uint16_t pwm_on, uint16_t pwm_off) {

        this->_set_pwm(led_n, pwm_on, pwm_off);
    }

};

#endif //PCA9685_H
