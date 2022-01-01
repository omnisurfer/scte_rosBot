//
// Created by user on 4/9/21.
//

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>

#include "i2c_linux/i2c_linux.h"

/*
 * @brief   Performs either writing or reading operation from I2C device
 *
 * @param   [in]        context device context
 * @param   [in/out]    message: message contains buffer with information on
 *                      how to store or retrieve information.
 *                      If message->write is true, data will be stored, otherwise read
 *
 * @returns             1 on success, otherwise 0
 */

static int i2c_operation(const context_t* context, message_t *message) {
    // check if connected
    if (!context->device) {
        return 0;
    }

    struct i2c_smbus_ioctl_data smbus_args;
    union i2c_smbus_data smbus_data;
    int result = 1;

    smbus_args.read_write = message->operation_is_write ? I2C_SMBUS_WRITE : I2C_SMBUS_READ;
    smbus_args.command = message->i2c_register;
    smbus_args.size = I2C_SMBUS_BYTE_DATA;
    smbus_args.data = &smbus_data;

    size_t data_length = message->buffer->size;

    for(size_t i=0; i < data_length; i++) {
        smbus_data.byte = message->buffer->bytes[i];
        smbus_args.command = message->i2c_register + i;
        if(ioctl(context->device, I2C_SMBUS, &smbus_args) < 0) {
            return 0;
        }
        if (!message->operation_is_write) {
            message->buffer->bytes[i] = smbus_data.byte;
            message->buffer->size = i + 1;
        }
    }

    return result;
}

int i2c_send(const context_t *context, buffer_t *data, uint8_t register_address) {

    message_t message = {
            .buffer = data,
            .i2c_register = register_address,
            .operation_is_write = 1,
    };

    return i2c_operation(context, &message);
}

int i2c_recv(const context_t *context, buffer_t *data, uint8_t register_address) {

    message_t message = {
            .buffer = data,
            .i2c_register = register_address,
            .operation_is_write = 0
    };

    return i2c_operation(context, &message);
}

int i2c_dev_open(context_t *context, int device_number, int slave_address) {
    char filename[64];
    sprintf(filename, "/dev/i2c-%d", device_number);

    if((context->device = open(filename, O_RDWR)) < 0) {
        //printf("Failed to open /dev/i2c-%d: %s\n", device_number, strerror(errno));
        return 0;
    }

    if(ioctl(context->device, I2C_SLAVE, slave_address) < 0) {
        //printf("Failed to acquire bus access at %d", slave_address);
        return 0;
    }

    //printf("/dev/i2c-%d opened\r\n", device_number);

    return 1;
}

int i2c_is_connected(context_t *context) {
    // Device failed to open
    if(!context || !context->device) {
        return 0;
    }

    // Check if slave respondes to reads
    uint8_t temp[1];
    buffer_t buffer = {
            .bytes = temp,
            .size = 1
    };

    message_t message = {
            .buffer = &buffer,
            .operation_is_write = 0,
            .i2c_register = 0
    };

    return i2c_operation(context, &message);
}

void i2c_dev_close(context_t *context, int device_number) {
    //printf("/dev/i2c-%d closed\r\n", device_number);

    close(context->device);
    context->device = 0;
}

int main(int argc, char* argv[]) {
    printf("i2c_linux\n");

    context_t context = {0};
    int device_id = 0;

    uint8_t outputBuffer[MAX_BUFFER_SIZE] = {0};

    uint8_t inputBuffer[] = "Hello World";

    int device_open = i2c_dev_open(&context, device_id, 0x03);

    if(!device_open) {
        printf("device_open failed\n");
    }

    int is_connected = i2c_is_connected(&context);

    if(!is_connected) {
        printf("is_connected failed\n");
    }

    buffer_t inputMessage = {
            .bytes = inputBuffer,
            .size = sizeof(inputBuffer)
    };

    int message_sent = i2c_send(&context, &inputMessage, 0x00);

    if(!message_sent) {
        printf("message_sent failed\n");
    }

    buffer_t outputMessage = {
            .bytes = outputBuffer,
            .size = sizeof(outputBuffer)
    };

    int message_received = i2c_recv(&context, &outputMessage, 0x00);

    if(!message_received) {
        printf("message_received failed\n");
    }

    i2c_dev_close(&context, device_id);

    printf("end of i2c_linux");

    return 0;
}