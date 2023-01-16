//
// Created by user on 4/9/21.
//

// I2C based on code from https://git.amongbytes.com/kris/i2c-stub-toy/src/branch/master/src/i2c.c

// The Linux Kernel:
// https://01.org/linuxgraphics/gfx-docs/drm/driver-api/i2c.html
// https://www.kernel.org/doc/Documentation/i2c/dev-interface
// https://stackoverflow.com/questions/18874962/linux-device-driver-file-operations-it-is-possible-to-have-race-conditions
// https://stackoverflow.com/questions/16467729/i2c-concurrent-access-on-linux-mutex
// https://stackoverflow.com/questions/16728587/i2c-driver-in-linux
// https://www.kernel.org/doc/html/latest/i2c/dev-interface.html

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef I2C_LINUX_H_
#define I2C_LINUX_H_

// max message size in bytes. represents number of registers accessible in i2c-stub
#define MAX_BUFFER_SIZE 256U

// buffer struct
typedef struct buffer_t {
    uint8_t *bytes;
    size_t size;
}buffer_t;

// Device connection context
typedef struct context_t {
    int device;
} context_t;

// Stores a buffer message to write or that was read
typedef struct message_t {
    buffer_t *buffer;          // data to be sent, or where received data is stored
    uint8_t i2c_register;       // register to read or write to?
    int operation_is_write; // indicate operation is write = 1, read = 0
} message_t;

/*
 * @brief   Opens a persistent connection to I2C slave device
 *
 * @param   [out]   context device context
 * @param   [in]    device_num character device which should be used for connection
 *                  For example, if device is /dev/i2c-8, then device_num=8
 * @param   [in]    slave_address of the I2C slave located on the device
 * @returns         1 on success and sets 'context', 0 otherwise
 */
int i2c_dev_open(context_t *context, int device_num, int slave_id);

/*
 * @brief   Closes connection to I2C slave
 *
 * @param   [in]    slave_addr address of the I2C slave.
 *
 * @returns         none
 */
void i2c_dev_close(context_t *context, int device_id);

/*
 * @brief   Checks if connection is opened and I2C slave is responding to commands
 *
 * @param   [in]    context device context
 * @returns         1 on success, 0 otherwise
 */
int i2c_is_connected(context_t *context);

/*
 * @brief   Sends data to I2C device
 *
 * @param   [in]    context device context
 * @param   [in]    data - buffer with a data to send
 * @param   [in]    register_address - what register to write the value at
 * @returns         1 on success, 0 otherwise
 */
int i2c_send(const context_t *context, buffer_t *data, uint8_t register_address);

/*
 * @brief   Receives data from I2C slave
 *
 * @param   [in]    context device context
 * @param   [out]   data - buffer to store received data in. Buffer
 *                  must be big enough to store data bytes.
 * @param   [in]    register_address - what register to write the value at
 * @returns         1 on success, 0 otherwise
 */
int i2c_recv(const context_t *context, buffer_t *data,  uint8_t register_address);

#endif //I2C_LINUX_H_

#ifdef __cplusplus
}
#endif
