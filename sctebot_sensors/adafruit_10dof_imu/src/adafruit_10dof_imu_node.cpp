//
// Created by user on 9/12/21.
//
#include <iostream>
#include <iomanip>
#include <string>
#include <cmath>
#include <cstring>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/Vector3.h>

#include "adafruit_10dof_imu.h"

namespace logging = boost::log;

// region Data Structs

std::mutex imu_data_mutex;
struct imu_data_t {
    float temperature;

    geometry_msgs::Quaternion orientation;
    double orientation_covariance[9];

    geometry_msgs::Vector3 angular_velocity;
    double angular_velocity_covariance[9];

    geometry_msgs::Vector3 linear_acceleration;
    double linear_acceleration_covariance[9];
} imu_data;


std::mutex pressure_data_mutex;
struct pressure_data_t {
    double fluid_absolute_pressure_pascals;
    double fluid_sea_level_pressure_pascals;
    double variance;
} pressure_data;


std::mutex altitude_range_mutex;
struct altitude_range_data_t {
    float range;
} altitude_range_data;


std::mutex magnetometer_data_mutex;
struct magnetometer_data_t {
    geometry_msgs::Vector3 magnetic_field_tesla;
    float temperature;
    double magnetic_field_covariance[9];
} magnetometer_data;


std::mutex temperature_data_mutex;
struct temperature_data_t {
    double temperature;
    double variance;
} temperature_data;

// endregion

// region Worker Variables

//pressure_temperature_range_publisher
std::condition_variable ros_press_temp_range_publisher_worker_publish_cv;
std::mutex ros_press_temp_range_publisher_worker_publish_cv_mutex;

bool run_ros_press_temp_range_publisher_worker = true;
std::mutex run_ros_press_temp_range_publisher_worker_mutex;
std::thread ros_press_temp_range_publisher_thread;

//imu_publisher
std::condition_variable ros_imu_publisher_worker_publish_cv;
std::mutex ros_imu_publisher_worker_publish_cv_mutex;

bool run_ros_imu_publisher_worker = true;
std::mutex run_ros_imu_publisher_worker_mutex;
std::thread ros_imu_publisher_thread;

//mag_publisher
std::condition_variable ros_magnetometer_publisher_worker_publish_cv;
std::mutex ros_magnetometer_publisher_worker_publish_cv_mutex;

bool run_ros_magnetometer_publisher_worker = true;
std::mutex run_ros_magnetometer_publisher_worker_mutex;
std::thread ros_magnetometer_publisher_thread;

// endregion

// region Measurement Handlers

void handle_bmp180_pressure_measurements(float temperature, float pressure) {

    BOOST_LOG_TRIVIAL(debug)
            << "[bmp180]"
            << "\t\t temp (C): " << std::fixed << std::setprecision(2) << temperature
            << "\t\t pressure (Pa): " << std::fixed << std::setprecision(2) << pressure;

    double absolute_altitude;
    double pressure_sea_level = 1013.25 * 100;

    double pressure_ratio = pressure / pressure_sea_level;
    double exponent_ratio = 1/5.255;

    absolute_altitude = 44330 * (1 - pow(pressure_ratio, exponent_ratio));

    double sea_level_pressure;

    sea_level_pressure = pressure / pow(1 - absolute_altitude/44330, 5.255);

    BOOST_LOG_TRIVIAL(debug)
            << "[bmp180]"
            << "\t\t abs alt (m): " << std::fixed << std::setprecision(2) << absolute_altitude
            << "\t\t sea (Pa): " << std::fixed << std::setprecision(2) << sea_level_pressure;

    std::lock_guard<std::mutex> pressure_guard(pressure_data_mutex);
    pressure_data.fluid_absolute_pressure_pascals = pressure;
    pressure_data.fluid_sea_level_pressure_pascals = sea_level_pressure;
    pressure_data.variance = 0.0;

    std::lock_guard<std::mutex> temperature_guard(temperature_data_mutex);
    temperature_data.temperature = temperature;
    temperature_data.variance = 0.0;

    std::lock_guard<std::mutex> range_guard(altitude_range_mutex);
    altitude_range_data.range = float(absolute_altitude);

    ros_press_temp_range_publisher_worker_publish_cv.notify_all();
}

void handle_l3gd20_gyro_measurements(float temperature, float r_x, float r_y, float r_z) {

    BOOST_LOG_TRIVIAL(debug)
            << "[l3gd20 gyro]"
            << "\t temp (C): " << std::fixed << std::setprecision(2) << temperature
            << "\t x_dps: " << std::fixed << std::setprecision(2) << r_x
            << "\t y_dps: " << std::fixed << std::setprecision(2) << r_y
            << "\t z_dps: " << std::fixed << std::setprecision(2) << r_z;

    std::unique_lock<std::mutex> imu_guard(imu_data_mutex);
    imu_data.temperature = temperature;

    imu_data.angular_velocity.x = r_x;
    imu_data.angular_velocity.y = r_y;
    imu_data.angular_velocity.z = r_z;
    imu_guard.unlock();

    // note, publish occurs on the accel update which hopefully will also include the gyro info
    // since its update occurs before the accel
    // ros_imu_publisher_worker_publish_cv.notify_all();
}

void handle_lsm303dlhc_accel_measurements(float x_gs, float y_gs, float z_gs) {

    BOOST_LOG_TRIVIAL(debug)
            << "[lsm303 accel]"
            << "\t x_gs: " << std::fixed << std::setprecision(2) << x_gs
            << "\t y_gs: " << std::fixed << std::setprecision(2) << y_gs
            << "\t z_gs: " << std::fixed << std::setprecision(2) << z_gs;

    std::unique_lock<std::mutex> imu_guard(imu_data_mutex);

    imu_data.linear_acceleration.x = x_gs;
    imu_data.linear_acceleration.y = y_gs;
    imu_data.linear_acceleration.z = z_gs;
    imu_guard.unlock();

    // note, publish occurs on the accel update which hopefully will also include the gyro info
    // since its update occurs before the accel
    ros_imu_publisher_worker_publish_cv.notify_all();
}

void handle_lsm303dlhc_mag_measurements(float temperature_deg_c, float x_ga, float y_ga, float z_ga) {

    BOOST_LOG_TRIVIAL(debug)
            << "[lsm303 mag]"
            << "\t temp (C): " << std::fixed << std::setprecision(2) << temperature_deg_c
            << "\t x_ga: " << std::fixed << std::setprecision(2) << x_ga
            << "\t y_ga: " << std::fixed << std::setprecision(2) << y_ga
            << "\t z_ga: " << std::fixed << std::setprecision(2) << z_ga;

    std::unique_lock<std::mutex> magnetometer_guard(magnetometer_data_mutex);
    magnetometer_data.temperature = temperature_deg_c;
    magnetometer_data.magnetic_field_tesla.x = x_ga;
    magnetometer_data.magnetic_field_tesla.y = y_ga;
    magnetometer_data.magnetic_field_tesla.z = z_ga;
    magnetometer_guard.unlock();

    ros_magnetometer_publisher_worker_publish_cv.notify_all();
}

// endregion

// region Measurement Publisher Workers

void ros_pressure_temperature_and_range_publisher_worker(const ros::Publisher& atm_pressure_publisher,
                                                         const ros::Publisher& sea_lvl_pressure_publisher,
                                                         const ros::Publisher& atm_temperature_publisher,
                                                         const ros::Publisher& atm_altitude_publisher
                                                        ) {

    BOOST_LOG_TRIVIAL(debug) << "ros_pressure_temperature_and_range_publisher_worker starting...";

    std::unique_lock<std::mutex> run_lock(run_ros_press_temp_range_publisher_worker_mutex);
    while(run_ros_press_temp_range_publisher_worker) {
        run_lock.unlock();

        std::unique_lock<std::mutex> publish_lock(ros_press_temp_range_publisher_worker_publish_cv_mutex);
        ros_press_temp_range_publisher_worker_publish_cv.wait(publish_lock);
        publish_lock.unlock();

        // copy over the data
        std::unique_lock<std::mutex> pressure_guard(pressure_data_mutex);
        double pressure =  pressure_data.fluid_absolute_pressure_pascals;
        double sea_level_pressure = pressure_data.fluid_sea_level_pressure_pascals;
        pressure_guard.unlock();

        std::unique_lock<std::mutex> temperature_guard(temperature_data_mutex);
        double temperature = temperature_data.temperature;
        temperature_guard.unlock();

        std::unique_lock<std::mutex> range_guard(altitude_range_mutex);
        float altitude_range = altitude_range_data.range;
        range_guard.unlock();

        if(false) {
            std::cout << "pressure " << pressure <<
                        " sea press " << sea_level_pressure
                        << " temp " << temperature
                        << " alt range " << altitude_range
                        << std::endl;
        }

        ros::Time message_time = ros::Time::now();

        sensor_msgs::FluidPressure atm_pressure_msg = sensor_msgs::FluidPressure();
        atm_pressure_msg.header.frame_id = "atm_frame";
        atm_pressure_msg.header.seq = 0;
        atm_pressure_msg.header.stamp = message_time;
        atm_pressure_msg.fluid_pressure = pressure;
        atm_pressure_msg.variance = Bmp180Pressure::pressure_variance;

        sensor_msgs::FluidPressure sea_lvl_pressure_msg = sensor_msgs::FluidPressure();
        sea_lvl_pressure_msg.header.frame_id = "sea_lvl_frame";
        sea_lvl_pressure_msg.header.seq = 0;
        sea_lvl_pressure_msg.header.stamp = message_time;
        sea_lvl_pressure_msg.fluid_pressure = sea_level_pressure;
        sea_lvl_pressure_msg.variance = Bmp180Pressure::pressure_variance;

        sensor_msgs::Temperature atm_temperature_msg = sensor_msgs::Temperature();
        atm_temperature_msg.header.frame_id = "temperature_frame";
        atm_temperature_msg.header.seq = 0;
        atm_temperature_msg.header.stamp = message_time;
        atm_temperature_msg.temperature = temperature;
        atm_temperature_msg.variance = Bmp180Pressure::temperature_variance;

        sensor_msgs::Range atm_altitude_msg = sensor_msgs::Range();
        atm_altitude_msg.header.frame_id = "atm_altitude_msg";
        atm_altitude_msg.header.seq = 0;
        atm_altitude_msg.header.stamp = message_time;
        atm_altitude_msg.range = altitude_range;
        atm_altitude_msg.field_of_view = 90.0;
        atm_altitude_msg.min_range = -1000;
        atm_altitude_msg.max_range = 30000;

        atm_pressure_publisher.publish(atm_pressure_msg);
        sea_lvl_pressure_publisher.publish(sea_lvl_pressure_msg);
        atm_temperature_publisher.publish(atm_temperature_msg);
        atm_altitude_publisher.publish(atm_altitude_msg);

        run_lock.lock();
    }
    BOOST_LOG_TRIVIAL(debug) << "ros_pressure_temperature_and_range_publisher_worker exiting...";
}

void ros_imu_publisher_worker(const ros::Publisher& imu_publisher) {

    BOOST_LOG_TRIVIAL(debug) << "ros_imu_data_publisher_worker starting...";

    std::unique_lock<std::mutex> run_lock(run_ros_imu_publisher_worker_mutex);
    while(run_ros_imu_publisher_worker) {
        run_lock.unlock();

        std::unique_lock<std::mutex> publish_lock(ros_imu_publisher_worker_publish_cv_mutex);
        ros_imu_publisher_worker_publish_cv.wait(publish_lock);
        publish_lock.unlock();

        // copy over the data
        std::unique_lock<std::mutex> imu_guard(imu_data_mutex);
        float temperature = imu_data.temperature;
        geometry_msgs::Quaternion orientation = imu_data.orientation;
        geometry_msgs::Vector3 angular_velocity = imu_data.angular_velocity;
        geometry_msgs::Vector3 linear_acceleration = imu_data.linear_acceleration;
        imu_guard.unlock();

        if(false) {
            std::cout << "i_t " << temperature
                        << " o_x " << orientation.x
                        << " o_y " << orientation.y
                        << " o_z " << orientation.z
                        << " o_w " << orientation.w
                        << " av_x " << angular_velocity.x
                        << " av_y " << angular_velocity.y
                        << " av_z " << angular_velocity.z
                        << " aa_x " << linear_acceleration.x
                        << " aa_y " << linear_acceleration.y
                        << " aa_z " << linear_acceleration.z
                        << std::endl;
        }

        sensor_msgs::Imu imu_msg = sensor_msgs::Imu();
        imu_msg.header.frame_id = "imu_frame";
        imu_msg.header.seq = 0;
        imu_msg.header.stamp = ros::Time::now();

        // TODO can I calculate quaternion with given info?
        // orientation
        orientation.x = 0.354;
        orientation.y = 0.354;
        orientation.z = 0.146;
        orientation.w = 0.854;

        imu_msg.orientation = orientation;

        imu_msg.orientation_covariance[0] = Lsm303DlhcAccelerometer::orientation_field_covariance[0];
        imu_msg.orientation_covariance[1] = Lsm303DlhcAccelerometer::orientation_field_covariance[1];
        imu_msg.orientation_covariance[2] = Lsm303DlhcAccelerometer::orientation_field_covariance[2];

        imu_msg.orientation_covariance[3] = Lsm303DlhcAccelerometer::orientation_field_covariance[3];
        imu_msg.orientation_covariance[4] = Lsm303DlhcAccelerometer::orientation_field_covariance[4];
        imu_msg.orientation_covariance[5] = Lsm303DlhcAccelerometer::orientation_field_covariance[5];

        imu_msg.orientation_covariance[6] = Lsm303DlhcAccelerometer::orientation_field_covariance[6];
        imu_msg.orientation_covariance[7] = Lsm303DlhcAccelerometer::orientation_field_covariance[7];
        imu_msg.orientation_covariance[8] = Lsm303DlhcAccelerometer::orientation_field_covariance[8];

        // linear accel
        linear_acceleration.x *= GRAVITY_MS_S;
        linear_acceleration.y *= GRAVITY_MS_S;
        linear_acceleration.z *= GRAVITY_MS_S;

        imu_msg.linear_acceleration = linear_acceleration;

        imu_msg.linear_acceleration_covariance[0] = Lsm303DlhcAccelerometer::acceleration_field_covariance[0];
        imu_msg.linear_acceleration_covariance[1] = Lsm303DlhcAccelerometer::acceleration_field_covariance[1];
        imu_msg.linear_acceleration_covariance[2] = Lsm303DlhcAccelerometer::acceleration_field_covariance[2];

        imu_msg.linear_acceleration_covariance[3] = Lsm303DlhcAccelerometer::acceleration_field_covariance[3];
        imu_msg.linear_acceleration_covariance[4] = Lsm303DlhcAccelerometer::acceleration_field_covariance[4];
        imu_msg.linear_acceleration_covariance[5] = Lsm303DlhcAccelerometer::acceleration_field_covariance[5];

        imu_msg.linear_acceleration_covariance[6] = Lsm303DlhcAccelerometer::acceleration_field_covariance[6];
        imu_msg.linear_acceleration_covariance[7] = Lsm303DlhcAccelerometer::acceleration_field_covariance[7];
        imu_msg.linear_acceleration_covariance[8] = Lsm303DlhcAccelerometer::acceleration_field_covariance[8];

        // angular velocity
        angular_velocity.x *= DEG_TO_RAD;
        angular_velocity.y *= DEG_TO_RAD;
        angular_velocity.z *= DEG_TO_RAD;

        imu_msg.angular_velocity = angular_velocity;

        imu_msg.angular_velocity_covariance[0] = L3gd20Gyro::angular_field_covariance[0];
        imu_msg.angular_velocity_covariance[1] = L3gd20Gyro::angular_field_covariance[1];
        imu_msg.angular_velocity_covariance[2] = L3gd20Gyro::angular_field_covariance[2];

        imu_msg.angular_velocity_covariance[3] = L3gd20Gyro::angular_field_covariance[3];
        imu_msg.angular_velocity_covariance[4] = L3gd20Gyro::angular_field_covariance[4];
        imu_msg.angular_velocity_covariance[5] = L3gd20Gyro::angular_field_covariance[5];

        imu_msg.angular_velocity_covariance[6] = L3gd20Gyro::angular_field_covariance[6];
        imu_msg.angular_velocity_covariance[7] = L3gd20Gyro::angular_field_covariance[7];
        imu_msg.angular_velocity_covariance[8] = L3gd20Gyro::angular_field_covariance[8];

        imu_publisher.publish(imu_msg);

        run_lock.lock();
    }
    BOOST_LOG_TRIVIAL(debug) << "ros_imu_data_publisher_worker exiting...";
}

void ros_magnetometer_publisher_worker(const ros::Publisher& magnetometer_publisher) {

    BOOST_LOG_TRIVIAL(debug) << "ros_magnetometer_publisher_worker starting...";

    std::unique_lock<std::mutex> run_lock(run_ros_magnetometer_publisher_worker_mutex);
    while(run_ros_magnetometer_publisher_worker) {
        run_lock.unlock();

        std::unique_lock<std::mutex> publish_lock(ros_magnetometer_publisher_worker_publish_cv_mutex);
        ros_magnetometer_publisher_worker_publish_cv.wait(publish_lock);
        publish_lock.unlock();

        // copy over the data
        std::unique_lock<std::mutex> mag_guard(magnetometer_data_mutex);
        float temperature = magnetometer_data.temperature;
        geometry_msgs::Vector3 magnetic_field_tesla = magnetometer_data.magnetic_field_tesla;
        mag_guard.unlock();

        if(false) {
            std::cout << "m_t " << temperature
                      << " m_x " << magnetic_field_tesla.x
                      << " m_y " << magnetic_field_tesla.y
                      << " m_z " << magnetic_field_tesla.z
                      << std::endl;
        }

        magnetic_field_tesla.x *= 10000;
        magnetic_field_tesla.y *= 10000;
        magnetic_field_tesla.z *= 10000;

        sensor_msgs::MagneticField mag_msg = sensor_msgs::MagneticField();
        mag_msg.header.frame_id = "mag_frame";
        mag_msg.header.seq = 0;
        mag_msg.header.stamp = ros::Time::now();
        geometry_msgs::Vector3 tesla_to_gauss;
        mag_msg.magnetic_field = magnetic_field_tesla;

        mag_msg.magnetic_field_covariance[0] = Lsm303DlhcMagnetometer::magnetic_field_covariance[0];
        mag_msg.magnetic_field_covariance[1] = Lsm303DlhcMagnetometer::magnetic_field_covariance[1];
        mag_msg.magnetic_field_covariance[2] = Lsm303DlhcMagnetometer::magnetic_field_covariance[2];

        mag_msg.magnetic_field_covariance[3] = Lsm303DlhcMagnetometer::magnetic_field_covariance[3];
        mag_msg.magnetic_field_covariance[4] = Lsm303DlhcMagnetometer::magnetic_field_covariance[4];
        mag_msg.magnetic_field_covariance[5] = Lsm303DlhcMagnetometer::magnetic_field_covariance[5];

        mag_msg.magnetic_field_covariance[6] = Lsm303DlhcMagnetometer::magnetic_field_covariance[6];
        mag_msg.magnetic_field_covariance[7] = Lsm303DlhcMagnetometer::magnetic_field_covariance[7];
        mag_msg.magnetic_field_covariance[8] = Lsm303DlhcMagnetometer::magnetic_field_covariance[8];

        magnetometer_publisher.publish(mag_msg);

        run_lock.lock();
    }
    BOOST_LOG_TRIVIAL(debug) << "ros_magnetometer_publisher_worker exiting...";
}
// endregion

/* IMU Sensor Msg */
//http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html

int main(int argc, char* argv[]) {

    logging::core::get()->set_filter
    (
        logging::trivial::severity >= logging::trivial::info
    );

    std::cout << "Hello World adafruit 10dof IMU Node 2253" << std::endl;

    ros::init(argc, argv, "adafruit_10dof_node");

    ros::NodeHandle ros_node_handle;

    ros::Publisher imu_publisher = ros_node_handle.advertise<sensor_msgs::Imu>("ada_imu", 1000);
    ros::Publisher magnetometer_publisher = ros_node_handle.advertise<sensor_msgs::MagneticField>("ada_mag", 1000);

    ros::Publisher atm_pressure_publisher = ros_node_handle.advertise<sensor_msgs::FluidPressure>("ada_atm_pressure", 100);
    ros::Publisher sea_lvl_pressure_publisher = ros_node_handle.advertise<sensor_msgs::FluidPressure>("ada_sea_lvl_pressure", 100);

    ros::Publisher atm_temperature_publisher = ros_node_handle.advertise<sensor_msgs::Temperature>("ada_atm_temperature", 100);
    ros::Publisher atm_altitude_publisher = ros_node_handle.advertise<sensor_msgs::Range>("ada_atm_altitude", 100);

    ros::Rate loop_rate(10);

    ros_press_temp_range_publisher_thread = std::thread(
            ros_pressure_temperature_and_range_publisher_worker,
            atm_pressure_publisher,
            sea_lvl_pressure_publisher,
            atm_temperature_publisher,
            atm_altitude_publisher
            );

    ros_imu_publisher_thread = std::thread(
            ros_imu_publisher_worker,
            imu_publisher
            );

    ros_magnetometer_publisher_thread = std::thread(
            ros_magnetometer_publisher_worker,
            magnetometer_publisher
            );

#if 0
    int count = 0;
    while(ros::ok()) {

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }
#endif

#if 1
    int i2c_bus_number = 0;
    //int i2c_device_address = 0x77;

    // lame way to do this but good enough for debug
    if(argv[1]) {
        if (!memcmp("-d", argv[1], 2)) {
            // i2c_bus_number = atoi(argv[2]);

            char* p_end;
            i2c_bus_number = (int)std::strtol(argv[2], &p_end, 10);

            if (*p_end) {
                //not sure what to do in this case
            }
        }
    }

    AdaFruit10DoFImu adaFruit10DoFImu = AdaFruit10DoFImu();

    adaFruit10DoFImu.init_device(
            i2c_bus_number,
            handle_bmp180_pressure_measurements,
            handle_l3gd20_gyro_measurements,
            handle_lsm303dlhc_accel_measurements,
            handle_lsm303dlhc_mag_measurements
    );

    adaFruit10DoFImu.run();
#endif

    std::cout << "press any key to exit..." << std::endl;

    std::cin.get();

    // pressure publisher
    std::unique_lock<std::mutex> run_ros_pressure_publisher_worker_lock(run_ros_press_temp_range_publisher_worker_mutex);
    run_ros_press_temp_range_publisher_worker = false;
    run_ros_pressure_publisher_worker_lock.unlock();
    ros_press_temp_range_publisher_worker_publish_cv.notify_all();

    if(ros_press_temp_range_publisher_thread.joinable()) {
        ros_press_temp_range_publisher_thread.join();
    }

    // imu publisher
    std::unique_lock<std::mutex> run_ros_imu_publisher_worker_lock(run_ros_imu_publisher_worker_mutex);
    run_ros_imu_publisher_worker = false;
    run_ros_imu_publisher_worker_lock.unlock();
    ros_imu_publisher_worker_publish_cv.notify_all();

    if(ros_imu_publisher_thread.joinable()) {
        ros_imu_publisher_thread.join();
    }

    // magnetometer publisher
    std::unique_lock<std::mutex> run_ros_magnetometer_publisher_worker_lock(run_ros_magnetometer_publisher_worker_mutex);
    run_ros_magnetometer_publisher_worker = false;
    run_ros_magnetometer_publisher_worker_lock.unlock();
    ros_magnetometer_publisher_worker_publish_cv.notify_all();

    if(ros_magnetometer_publisher_thread.joinable()) {
        ros_magnetometer_publisher_thread.join();
    }

    return 0;
}
