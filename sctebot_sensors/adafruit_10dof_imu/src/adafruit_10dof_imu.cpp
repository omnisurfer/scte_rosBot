//
// Created by user on 4/9/21.
//

#include "adafruit_10dof_imu.h"

AdaFruit10DoFImu::AdaFruit10DoFImu() {

#if 0
#if ENABLE_BMP180_PRESSURE_DEVICE
    bmp180DeviceHandle = new Bmp180Pressure();
#endif

#if ENABLE_L3GD20_GYRO_DEVICE
    l3gd20GyroDeviceHandle = new L3gd20Gyro();
#endif

#if ENABLE_LSM303DLHC_ACCEL_DEVICE
    lsm303DlhcAccelDeviceHandle = new Lsm303DlhcAccelerometer();
#endif

#if ENABLE_LSM303DLHC_MAG_DEVICE
    lsm303DlhcMagDeviceHandle = new Lsm303DlhcMagnetometer();
#endif
#endif
}