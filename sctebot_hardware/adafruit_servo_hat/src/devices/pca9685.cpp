//
// Created by user on 12/17/21.
//
#include "pca9685.h"

int Pca9685LEDController::_init_device() {

    std::string device_name = this->_device_name;

    buffer_t inbound_message;
    buffer_t outbound_message;
    uint8_t register_address;
    uint8_t control_reg[1] = {0};

    int init_ok = 1;

    // region Mode1 configuration
#if 1
    BOOST_LOG_TRIVIAL(debug) << device_name <<": Mode1 config" << std::endl;

    register_address = Pca9685LEDController::Addresses::Registers::MODE1;

    uint8_t mode1_register_data[1];

    inbound_message = {
            .bytes = mode1_register_data,
            .size = sizeof(mode1_register_data)
    };

    if(receive_i2c(&inbound_message, register_address)) {
        // nothing to do here
    }
    else {
        init_ok = 0;
    }

    mode1_register_data[0] =
            (Pca9685LEDController::BitMasks::Mode1::RESTART & ENABLE) |
            (Pca9685LEDController::BitMasks::Mode1::EXTCLK & DISABLE) |
            (Pca9685LEDController::BitMasks::Mode1::AI & DISABLE) |
            (Pca9685LEDController::BitMasks::Mode1::SLEEP & ENABLE) |
            (Pca9685LEDController::BitMasks::Mode1::SUB1 & DISABLE) |
            (Pca9685LEDController::BitMasks::Mode1::SUB2 & DISABLE) |
            (Pca9685LEDController::BitMasks::Mode1::SUB3 & DISABLE) |
            (Pca9685LEDController::BitMasks::Mode1::ALLCALL & DISABLE);

    outbound_message = {
            .bytes = mode1_register_data,
            .size = sizeof(mode1_register_data)
    };

    if(send_i2c(&outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(debug) << device_name <<": MODE1 register configure OK (b" << std::bitset<8>(mode1_register_data[0]) << ")";
    }
    else {
        BOOST_LOG_TRIVIAL(debug) << device_name <<": MODE1 register configuration failed (b" << std::bitset<8>(mode1_register_data[0]) << ")";
    }
#endif
    // endregion Mode1 configuration

    // region Mode2 configuration
#if 1
    BOOST_LOG_TRIVIAL(debug) << device_name <<": Mode2 config" << std::endl;

    register_address = Pca9685LEDController::Addresses::Registers::MODE2;

    uint8_t mode2_register_data[1];

    inbound_message = {
            .bytes = mode2_register_data,
            .size = sizeof(mode2_register_data)
    };

    if(receive_i2c(&inbound_message, register_address)) {
        // nothing to do here
    }
    else {
        init_ok = 0;
    }

    control_reg[0] =
            (Pca9685LEDController::BitMasks::Mode2::INVRT & DISABLE) |
            (Pca9685LEDController::BitMasks::Mode2::OCH & DISABLE) |
            (Pca9685LEDController::BitMasks::Mode2::OUTDRV & ENABLE) |
            (Pca9685LEDController::BitMasks::Mode2::OUTNE_LEDN_HIGH_Z & DISABLE) |
            (Pca9685LEDController::BitMasks::Mode2::OUTNE_LEDN_SET_1 & DISABLE);

    outbound_message = {
            .bytes = control_reg,
            .size = sizeof(control_reg)
    };


    if(send_i2c(&outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(debug) << device_name <<": MODE2 register configure OK (b" << std::bitset<8>(control_reg[0]) << ")";
    }
    else {
        BOOST_LOG_TRIVIAL(debug) << device_name <<": MODE2 register configuration failed (b" << std::bitset<8>(control_reg[0]) << ")";
    }
#endif
    // endregion

    // region Pre-scale configuration
#if 1
    /*
     * prescale value = round(osc_clock / 4096 * update_rate) - 1
     * osc_clock = 25MHz
     * update_rate = 50Hz
     */

    BOOST_LOG_TRIVIAL(debug) << device_name <<": pre-scale device" << std::endl;
    uint8_t prescale_value = round(osc_clock / (4096 * update_rate)) - 1;

    register_address = Pca9685LEDController::Addresses::Registers::PRE_SCALE;

    control_reg[0] = prescale_value;

    outbound_message = {
            .bytes = control_reg,
            .size = sizeof(control_reg)
    };

    if(send_i2c(&outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(debug) << device_name <<": PRESCALE register configure OK (b" << std::bitset<8>(control_reg[0]) << ")";
    }
    else {
        BOOST_LOG_TRIVIAL(debug) << device_name <<": PRESCALE register configuration failed (b" << std::bitset<8>(control_reg[0]) << ")";
    }
#endif
    // endregion

    // region Wake Device
#if 1
    BOOST_LOG_TRIVIAL(debug) << device_name <<": wake device" << std::endl;

    register_address = Pca9685LEDController::Addresses::Registers::MODE1;

    mode1_register_data[0] = mode1_register_data[0] & ~(
            Pca9685LEDController::BitMasks::Mode1::SLEEP
            );

    control_reg[0] = mode1_register_data[0];

    outbound_message = {
            .bytes = control_reg,
            .size = sizeof(control_reg)
    };

    if(send_i2c(&outbound_message, register_address)) {
        BOOST_LOG_TRIVIAL(debug) << device_name <<": MODE1 register configure OK (b" << std::bitset<8>(control_reg[0]) << ")";
    }
    else {
        BOOST_LOG_TRIVIAL(debug) << device_name <<": MODE1 register configuration failed (b" << std::bitset<8>(control_reg[0]) << ")";
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
#endif
    // endregion

    // region Restart device
#if 0
    this->_restart_device();
#endif
    // endregion

    std::lock_guard<std::mutex> run_lock(this->run_servo_status_thread_mutex);
    this->run_servo_status_thread = true;
    this->run_servo_status_thread_cv.notify_all();

    return init_ok;
}

void Pca9685LEDController::_servo_management_worker() {

    std::unique_lock<std::mutex> run_lock(this->run_servo_status_thread_mutex);
    this->run_servo_status_thread_cv.wait(run_lock);
    run_lock.unlock();

    run_lock.lock();
    while(this->run_servo_status_thread) {
        run_lock.unlock();

        this->_host_callback_function(1, 2);

        std::map<Pca9685LEDController::LEDn, float>::iterator led_map_iterator;
        std::map<Pca9685LEDController::LEDn, float> ledn_map_pwm_power_on_percentage;

        ledn_map_pwm_power_on_percentage = this->get_ledn_pwr_on_percent_map();

        for(led_map_iterator = ledn_map_pwm_power_on_percentage.begin(); led_map_iterator != ledn_map_pwm_power_on_percentage.end(); led_map_iterator++) {
            //std::cout << "ch #: " << led_map_iterator->first << " %: " << led_map_iterator->second << " ";

            this->_set_pwm(led_map_iterator->first, led_map_iterator->second);

        }
        //std::cout << std::endl;

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_sensor_update_period_ms));

        run_lock.lock();
    }
}

int Pca9685LEDController::receive_i2c(buffer_t *data, uint8_t register_address) {

    std::lock_guard<std::mutex> i2c_device_lock(_i2c_device_mutex);
    int status = i2c_recv(&_i2c_device_context, data, register_address);

    return status;
}

int Pca9685LEDController::send_i2c(buffer_t *data, uint8_t register_address) {

    std::lock_guard<std::mutex> i2c_device_lock(_i2c_device_mutex);
    int status = i2c_send(&_i2c_device_context, data, register_address);

    return status;
}

int Pca9685LEDController::open_i2c_dev(int device_number, int slave_address) {

    std::lock_guard<std::mutex> i2c_device_lock(_i2c_device_mutex);
    int status = i2c_dev_open(&_i2c_device_context, device_number, slave_address);

    return status;
}

int Pca9685LEDController::is_i2c_dev_connected() {

    std::lock_guard<std::mutex> i2c_device_lock(_i2c_device_mutex);
    int status = i2c_is_connected(&_i2c_device_context);

    return status;
}

void Pca9685LEDController::close_i2c_dev(int bus_number) {

    std::lock_guard<std::mutex> i2c_device_lock(_i2c_device_mutex);
    i2c_dev_close(&_i2c_device_context, bus_number);
}

void handle_servo_callback(int x, int y) {

    //std::cout << "servo callback called!" << std::endl;

    // Boost logging causing thread race conditions (all of them???)
    //BOOST_LOG_TRIVIAL(debug) << "handle_servo_callback " << x << " " << y;
    //BOOST_LOG_TRIVIAL(info) << "callback info message";

}

int main(int argc, char* argv[]) {

    ScteBotBoostLogger sctebot_boost_logger = ScteBotBoostLogger();
    sctebot_boost_logger.init_boost_logging();

    BOOST_LOG_TRIVIAL(debug) << "pca9685 debug message";

    std::unique_ptr<Pca9685LEDController> pca9685DeviceHandle;

    pca9685DeviceHandle.reset(new Pca9685LEDController());

    int i2c_bus_number = 0;
    int i2c_device_address = 0x40;
    int update_period_ms = 1000;

    bool init_ok = true;

    int op_pwm_max_count_cycle = 4095;
    float op_pwm_on_percent = 0.0;
    float op_pwm_min_limit_duty_cycle = 0.03;
    float op_pwm_max_limit_duty_cycle = 0.125;
    float op_pwm_min_operating_duty_cycle = 0.03;
    float op_pwm_max_operating_duty_cycle = 0.125;
    float op_pwm_on_delay = 0.0;

    // lame way to do this but good enough for debug
    if(argv[1]) {
        if (!memcmp("-d", argv[1], 2)) {

            char* p_end;
            i2c_bus_number = (int)std::strtol(argv[2], &p_end, 10);

            if (*p_end) {
                //not sure what to do in this case
            }
        }
    }

    pca9685DeviceHandle->config_device(
            i2c_bus_number,
            i2c_device_address,
            update_period_ms,
            "pca9685_servo",
            handle_servo_callback
            );

    if(!pca9685DeviceHandle->connect_to_device()) {
        std::cout << "pca9685 failed to connect" << std::endl;
        return 0;
    }
    else {

        pca9685DeviceHandle->init_device(
                op_pwm_max_count_cycle,
                op_pwm_on_delay,
                op_pwm_min_limit_duty_cycle,
                op_pwm_max_limit_duty_cycle,
                op_pwm_min_operating_duty_cycle,
                op_pwm_max_operating_duty_cycle
                );

        std::cout << "press e key to exit" << std::endl;

        char input[2];
        float pwm_delta = 0.1;

        while(true) {

            std::cin.get(input, 2);

            if(op_pwm_on_percent > 1.0) {
                op_pwm_on_percent = 1.0;
            }
            else if (op_pwm_on_percent < 0.0) {
                op_pwm_on_percent = 0.0;
            }

            if (input[0] == 'e') {
                return 0;
            }
            else if(input[0] == 'u') {

                //pca9685DeviceHandle->set_pwm(Pca9685LEDController::LED0,op_pwm_on_percent);
                pca9685DeviceHandle->set_pwm_DEBUG(Pca9685LEDController::LED0, op_pwm_on_percent);
                pca9685DeviceHandle->set_pwm_DEBUG(Pca9685LEDController::LED1, op_pwm_on_percent);

                op_pwm_on_percent += pwm_delta;

                std::cin.clear();
                std::cin.ignore();
            }
            else if(input[0] == 'd') {

                //pca9685DeviceHandle->set_pwm(Pca9685LEDController::LED0,op_pwm_on_percent);
                pca9685DeviceHandle->set_pwm_DEBUG(Pca9685LEDController::LED0, op_pwm_on_percent);
                pca9685DeviceHandle->set_pwm_DEBUG(Pca9685LEDController::LED1, op_pwm_on_percent);

                op_pwm_on_percent -= pwm_delta;

                std::cin.clear();
                std::cin.ignore();
            }
            else if (input[0] == 'm') {

                /*
                pca9685DeviceHandle->set_pwm_bool(Pca9685LEDController::LED0, true);
                pca9685DeviceHandle->set_pwm_bool(Pca9685LEDController::LED1, false);
                std::this_thread::sleep_for(std::chrono::milliseconds (2000));
                pca9685DeviceHandle->set_pwm_bool(Pca9685LEDController::LED0, false);
                pca9685DeviceHandle->set_pwm_bool(Pca9685LEDController::LED1, true);
                */

                std::cin.clear();
                std::cin.ignore();
            }
            else {
                std::cin.clear();
                std::cin.ignore();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

}
