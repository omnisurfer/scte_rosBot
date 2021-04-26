//
// Created by user on 4/9/21.
//

#ifndef BMP180_H
#define BMP180_H

// some code taken from my github:
// https://github.com/omnisurfer/IMUController_firmware_AtmelC/blob/master/IMU/src/app_components/sensors/bmp180/bmp180.c

class Bmp180 {

private:
    uint8_t control_measurement_register;

    bool sensor_calibration_read = false;
    bool sensor_configured = false;

    struct Bmp180CalibrationCoefficients_t {
        union {
            int16_t AC1;
            uint8_t AC1_ba[2];
        };

        union {
            int16_t AC2;
            uint8_t AC2_ba[2];
        };

        union {
            int16_t AC3;
            uint8_t AC3_ba[2];
        };

        union {
            uint16_t AC4;
            uint8_t AC4_ba[2];
        };

        union {
            uint16_t AC5;
            uint8_t AC5_ba[2];
        };

        union {
            uint16_t AC6;
            uint8_t AC6_ba[2];
        };

        union {
            int16_t B1;
            uint8_t B1_ba[2];
        };

        union {
            int16_t B2;
            uint8_t B2_ba[2];
        };

        union {
            int16_t MB;
            uint8_t MB_ba[2];
        };

        union {
            int16_t MC;
            uint8_t MC_ba[2];
        };

        union {
            int16_t MD;
            uint8_t MD_ba[2];
        };

    } Bmp180CalibrationCoefficients;

    struct Bmp180SharedCoefficients_t {
        union {
            float B5;
        };
    } Bmp180SharedCoefficients;

public:

    class Addresses {

    public:
        typedef enum CalibrationCoefficients_t {

            AC1 = 0xAAAB,
            AC2 = 0xACAD,
            AC3 = 0xAEAF,
            AC4 = 0xB0B1,
            AC5 = 0xB2B3,
            AC6 = 0xB4B5,
            B1 = 0xB6B7,
            B2 = 0xB8B9,
            MB = 0xBABB,
            MC = 0xBCBD,
            MD = 0xBEBF

        } CalibrationCoefficients;

        typedef enum DataRegisters_t {

            OUTPUT_XLSB = 0xF8,
            OUTPUT = 0xF6F7,
            CONTROL_MEASUREMENT = 0xF4,
            SOFT_RESET = 0xE0,
            CHIP_ID = 0xD0
        } DataRegisters;
    };

    class Commands {

    public:
        typedef enum MeasurementControl_t {
            TEMPERATURE = 0x2E,
            PRESSURE_OSS0 = 0x34,
            PRESSURE_OSS1 = 0x74,
            PRESSURE_OSS2 = 0xB4,
            PRESSURE_OSS3 = 0xF4
        } MeasurementControl;

        typedef enum SoftReset_t {
            POWER_ON_RESET = 0xB6
        } SoftReset;

        typedef enum ChipId_t {
            CHIP_ID = 0x55
        } ChipId;
    };

    class BitMasks {

    public:
        typedef enum ControlRegister_t {
            OSS_BITS = 0b11000000,
            SCO_BIT = 0b00100000,
            MEASUREMENT_CONTROL_BITS = 0b00011111
        } ControlRegister;
    };

    int init_bmp180_calibration_coefficients(char *bytes, uint8_t length) {
        std::cout << "init_bmp180_calibration_coefficients\n";

        if(length < 0 or length > 22) {
            return 0;
        }

        // assumes little endian
        Bmp180CalibrationCoefficients.AC1_ba[0] = bytes[0];
        Bmp180CalibrationCoefficients.AC1_ba[1] = bytes[1];

        Bmp180CalibrationCoefficients.AC2_ba[0] = bytes[2];
        Bmp180CalibrationCoefficients.AC2_ba[1] = bytes[3];

        Bmp180CalibrationCoefficients.AC3_ba[0] = bytes[4];
        Bmp180CalibrationCoefficients.AC3_ba[1] = bytes[5];

        Bmp180CalibrationCoefficients.AC4_ba[0] = bytes[6];
        Bmp180CalibrationCoefficients.AC4_ba[1] = bytes[7];

        Bmp180CalibrationCoefficients.AC5_ba[0] = bytes[8];
        Bmp180CalibrationCoefficients.AC5_ba[1] = bytes[9];

        Bmp180CalibrationCoefficients.AC6_ba[0] = bytes[10];
        Bmp180CalibrationCoefficients.AC6_ba[1] = bytes[11];

        Bmp180CalibrationCoefficients.B1_ba[0] = bytes[12];
        Bmp180CalibrationCoefficients.B1_ba[1] = bytes[13];

        Bmp180CalibrationCoefficients.B2_ba[0] = bytes[14];
        Bmp180CalibrationCoefficients.B2_ba[1] = bytes[15];

        Bmp180CalibrationCoefficients.MB_ba[0] = bytes[16];
        Bmp180CalibrationCoefficients.MB_ba[1] = bytes[17];

        Bmp180CalibrationCoefficients.MC_ba[0] = bytes[18];
        Bmp180CalibrationCoefficients.MC_ba[1] = bytes[19];

        Bmp180CalibrationCoefficients.MD_ba[0] = bytes[20];
        Bmp180CalibrationCoefficients.MD_ba[1] = bytes[21];

        sensor_calibration_read = true;

        return 1;
    }

    // Note, temperature must be read first, then pressure so that B5 value can be populated
    int calculate_temperature(uint16_t uncompensated_temperature, float &temperature) {

        if(!sensor_calibration_read) {
            std::cout << "sensor calibration not read yet\n";
            return 0;
        }

        // refer to formula found on page 15 of datasheet
        float _temperature = uncompensated_temperature;

        float X1 = (_temperature - (float)Bmp180CalibrationCoefficients.AC6) * ((float)Bmp180CalibrationCoefficients.AC5/32768);
        float X2 = ((float)Bmp180CalibrationCoefficients.MC * 2048) / (X1 + (float)Bmp180CalibrationCoefficients.MD);

        Bmp180SharedCoefficients.B5 = X1 + X2;

        // multiplying by 0.1 here to convert to degrees C instead of degrees 0.1C
        _temperature = static_cast<float>(((Bmp180SharedCoefficients.B5 + 8) / 16)) * 0.1;

        temperature = _temperature;

        std::cout << "calc temp (C): " << temperature << std::endl;

        return 1;

    }

    int calculate_pressure(uint16_t uncompensated_pressure, uint8_t uncompensated_pressure_xlsb, float &pressure) {

        if(!sensor_calibration_read) {
            std::cout << "sensor calibration not read yet\n";
            return 0;
        }

        // uint8_t ut_l = 0x23;
        // uint8_t ut_h = 0x5d;

        // long _pressure = ((ut_h << 16) + (ut_l << 8) + uncompensated_pressure_xlsb) >> (8 - 0);
        long _pressure = (uncompensated_pressure << 8) >> (8 - 0);
        _pressure += uncompensated_pressure_xlsb;

        // refer to formula found on page 15 of datasheet
        // all these operations in powers of 2 seem to be "hiding" bitshifts. wonder if the compiler will notice
        // this...
        float _p = 0.00f,
            B6 = Bmp180SharedCoefficients.B5 - 4000,

            X1 = ((float)Bmp180CalibrationCoefficients.B2 * ((B6 * B6) / 4096)) / 2048,
            X2 = ((float)Bmp180CalibrationCoefficients.AC2 * B6) / 2048,
            X3 = X1 + X2,
            B3 = (((float)Bmp180CalibrationCoefficients.AC1 * 4 + X3) + 2) / 4;

            X1 = ((float)Bmp180CalibrationCoefficients.AC3 * B6) / 8192;
            X2 = ((float)Bmp180CalibrationCoefficients.B1 * (B6 * B6 / 4096)) / 65536;
            X3 = ((X1 + X2) + 2) / 4;

            unsigned long B4 = (Bmp180CalibrationCoefficients.AC4 * (unsigned)(X3 * 32768)) / 32768;

            //note _B7 has a scaling option that is tied to the oversample (OSS). I am assuming 0 oversampling for now
            unsigned long B7 = ((unsigned)_pressure - B3) * 50000;

            if(B7 < 0x80000000) {
                _p = (B7 * 2) / B4;
            }
            else {
                _p = (B7 / B4) * 2;
            }

            X1 = (_p/256) * (_p/256);
            X1 = (X1 * 3038) / 65536;
            X2 = (-7357 * _p) / 65536;

            _p = _p + ((X1 + X2 + 3791) / 16);

            pressure = _p;

            std::cout << "calc pressure (Pa): " << pressure << std::endl;

        return 1;
    }
};

#endif //BMP180_H
