//
// Created by user on 12/17/21.
//

#include "pca9685.h"
#include "boost_logging.h"

void Pca9685LEDController::_servo_status_worker() {

    std::unique_lock<std::mutex> run_lock(this->run_servo_status_thread_mutex);
    this->run_servo_status_thread_cv.wait(run_lock);
    run_lock.unlock();

    run_lock.lock();
    while(this->run_servo_status_thread) {
        run_lock.unlock();

        this->_host_callback_function(1, 2);

        std::this_thread::sleep_for(std::chrono::milliseconds (this->_sensor_update_period_ms));

        run_lock.lock();
    }
}

void handle_servo_callback(int x, int y) {

    //std::cout << "servo callback called!" << std::endl;

    // Boost logging causing thread race conditions (all of them???)
    //BOOST_LOG_TRIVIAL(debug) << "handle_servo_callback " << x << " " << y;
    //BOOST_LOG_TRIVIAL(info) << "callback info message";

}

/*
void init_boost_logging() {

    boost::shared_ptr<logging::core> core = logging::core::get();

    boost::shared_ptr<sinks::text_ostream_backend> backend = boost::make_shared<sinks::text_ostream_backend>();

    //backend->add_stream(boost::make_shared<std::ofstream>("sample.log"));
    backend->add_stream(boost::shared_ptr< std::ostream >(&std::clog, boost::null_deleter()));

    typedef sinks::synchronous_sink<sinks::text_ostream_backend> sink_t;
    boost::shared_ptr<sink_t> sink(new sink_t(backend));
    core->add_sink(sink);
}
*/

int main(int argc, char* argv[]) {

    //init_boost_logging();
    //logging::add_common_attributes();

    //BOOST_LOG_TRIVIAL(debug) << "main x debug message";

    std::unique_ptr<Pca9685LEDController> pca9685DeviceHandle;

    pca9685DeviceHandle.reset(new Pca9685LEDController());

    int i2c_bus_number = 0;
    int i2c_device_address = 0x40;

    bool init_ok = true;

    pca9685DeviceHandle->config_device(
            i2c_bus_number,
            i2c_device_address,
            3000,
            "pca9685_servo",
            handle_servo_callback
            );

    if(pca9685DeviceHandle->connect_to_device()) {

        if(pca9685DeviceHandle->init_device()) {

        }
        else {
            init_ok = false;
        }

    }
    else {
        init_ok = false;
    }

    std::cout << "press any key to exit" << std::endl;
    std::cin.get();

}