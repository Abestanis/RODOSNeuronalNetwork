//
// Created by Sebastian Scholz on 28.03.2020.
//

#include "LSM9DS1.h"

#define RETURN_IF_HAL_ERROR(operation) do { \
    int32_t operationResult = operation; \
    if (operationResult < 0) { \
        return operationResult; \
    } \
} while(0)
#define HAS_FLAG(value, flag) ((value & flag) == flag)
#define HAS_MAGNETOMETER_DATA_FLAG 0b00001000U
#define HAS_TEMPERATURE_DATA_FLAG 0b00000100U
#define HAS_GYROSCOPE_DATA_FLAG 0b00000010U
#define HAS_ACCELEROMETER_DATA_FLAG 0b00000001U

#define READ_BIT (1U << 7U)
#define WRITE_MASK (~READ_BIT)
#define SENSOR_RESET 0b1U
#define AUTO_INCREMENT_ADDRESS 0b100U
#define DATA_RATE_952HZ 0b110U
#define DATA_RATE_476HZ 0b101U
#define ACCEL_SCALE_2G 0b00U
#define GYRO_SCALE_245DPS 0b00U
#define MAG_SCALE_4GAUSS 0b00U
#define ENABLE_ALL_AXIS 0b00111000
#define DISABLE_I2C 0b10000000U
#define SPI_ENABLE_READ 0b100U
#define MAGNETOMETER_CONTINUOUS_CONVERSION 0b00U
#define ACCEL_GYRO_ID 0x68
#define MAGNETOMETER_ID 0x3d


LSM9DS1::LSM9DS1(SPI_IDX spiId, GPIO_PIN spiClockPin, GPIO_PIN spiMisoPin, GPIO_PIN spiMosiPin,
                 GPIO_PIN accGyroSelectPin, GPIO_PIN magSelectPin) :
        spi(spiId, spiClockPin, spiMisoPin, spiMosiPin),
        chipSelectAccelGyro(accGyroSelectPin), chipSelectMag(magSelectPin) {
}

int32_t LSM9DS1::init() {
    RETURN_IF_HAL_ERROR(chipSelectAccelGyro.init(true, 1, 1));
    RETURN_IF_HAL_ERROR(chipSelectMag.init(true, 1, 1));
    RETURN_IF_HAL_ERROR(spi.init());
    RETURN_IF_HAL_ERROR(
            writeByte(chipSelectAccelGyro, SENSOR_CONTROL, AUTO_INCREMENT_ADDRESS | SENSOR_RESET));
    Thread::suspendCallerUntil(NOW() + 10 * MILLISECONDS);
    if (getChipId(chipSelectAccelGyro) != ACCEL_GYRO_ID) {
        return 0;
    }
    if (getChipId(chipSelectMag) != MAGNETOMETER_ID) {
        return 0;
    }
    return 1;
}

int32_t LSM9DS1::enableAccelerometer() {
    RETURN_IF_HAL_ERROR(writeByte(chipSelectAccelGyro, ACCEL_CONTROL5, ENABLE_ALL_AXIS));
    return writeByte(chipSelectAccelGyro,
            ACCEL_CONTROL6, (DATA_RATE_952HZ << 5U) | (ACCEL_SCALE_2G << 3U));
}

int32_t LSM9DS1::enableGyroscope() {
    return writeByte(chipSelectAccelGyro,
            GYRO_CONTROL1, (DATA_RATE_952HZ << 5U) | (GYRO_SCALE_245DPS << 3U));
}

int32_t LSM9DS1::enableMagnetometer() {
    RETURN_IF_HAL_ERROR(writeByte(
            chipSelectMag, SENSOR_CONTROL, MAGNETOMETER_CONTINUOUS_CONVERSION));
    return writeByte(chipSelectMag,
            MAG_CONTROL2, (MAG_SCALE_4GAUSS << 5U));
}

uint8_t LSM9DS1::getChipId(HAL_GPIO &chipSelect) {
    return readByte(chipSelect, DEVICE_ID);
}

uint8_t LSM9DS1::getStatus(HAL_GPIO &chipSelect) {
    return readByte(chipSelect, STATUS);
}

int32_t LSM9DS1::readAccelerometer(Int16Vec3d* data) {
    waitForStatus(chipSelectAccelGyro, HAS_ACCELEROMETER_DATA_FLAG);
    uint8_t buffer[6];
    RETURN_IF_HAL_ERROR(read(chipSelectAccelGyro, ACCEL_X_OUT, buffer, 6));
    data->x = bytesToInt16(&buffer[0]);
    data->y = bytesToInt16(&buffer[2]);
    data->z = bytesToInt16(&buffer[4]);
    return 1;
}

int32_t LSM9DS1::readGyroscope(LSM9DS1::Int16Vec3d* data) {
    waitForStatus(chipSelectAccelGyro, HAS_GYROSCOPE_DATA_FLAG);
    uint8_t buffer[6];
    RETURN_IF_HAL_ERROR(read(chipSelectAccelGyro, GYRO_X_OUT, buffer, 6));
    data->x = bytesToInt16(&buffer[0]);
    data->y = bytesToInt16(&buffer[2]);
    data->z = bytesToInt16(&buffer[4]);
    return 1;
}

int32_t LSM9DS1::readMagnetometer(LSM9DS1::Int16Vec3d* data) {
    waitForStatus(chipSelectMag, HAS_MAGNETOMETER_DATA_FLAG);
    uint8_t buffer[6];
    RETURN_IF_HAL_ERROR(read(chipSelectMag, MAG_X_OUT, buffer, 6));
    data->x = bytesToInt16(&buffer[0]);
    data->y = bytesToInt16(&buffer[2]);
    data->z = bytesToInt16(&buffer[4]);
    return 1;
}

int32_t LSM9DS1::readTemperature(int16_t* temperature) {
    waitForStatus(chipSelectAccelGyro, HAS_TEMPERATURE_DATA_FLAG);
    uint8_t data[2];
    RETURN_IF_HAL_ERROR(read(chipSelectAccelGyro, TEMPERATURE_OUT, data, 2));
    *temperature = bytesToInt16(data);
    return 1;
}

void LSM9DS1::waitForStatus(HAL_GPIO &chipSelect, uint8_t statusFlag) {
    while (!HAS_FLAG(getStatus(chipSelect), statusFlag)) {
        Thread::yield();
    }
}

int32_t LSM9DS1::read(
        HAL_GPIO &chipSelect, LSM9DS1Address address, uint8_t* buffer, size_t bufferSize) {
    ChipSelector selector(chipSelect);
    Thread::suspendCallerUntil(NOW() + 10 * MILLISECONDS);
    uint8_t addressByte = address | READ_BIT;
    RETURN_IF_HAL_ERROR(spi.write(&addressByte, 1));
    return spi.read(buffer, bufferSize);
}

uint8_t LSM9DS1::readByte(HAL_GPIO &chipSelect, LSM9DS1Address address) {
    uint8_t data = 0;
    read(chipSelect, address, &data, 1);
    return data;
}

int32_t LSM9DS1::write(
        HAL_GPIO &chipSelect, LSM9DS1Address address, uint8_t* buffer, size_t bufferSize) {
    ChipSelector selector(chipSelect);
    //    uint8_t data[5];
    //    data[0] = (address & WRITE_MASK);
    //    memcpy(&data[1], buffer, bufferSize);
    //    return spi.write(&data, bufferSize + 1);
    uint8_t addressByte = (address & WRITE_MASK);
    RETURN_IF_HAL_ERROR(spi.write(&addressByte, 1));
    return spi.write(buffer, bufferSize);
}

int32_t LSM9DS1::writeByte(HAL_GPIO &chipSelect, LSM9DS1Address address, uint8_t byte) {
    return write(chipSelect, address, &byte, 1);
}

int16_t LSM9DS1::bytesToInt16(const uint8_t* data) {
    uint8_t lowByte = data[0];
    uint16_t result = data[1];
    result <<= 8U;
    result |= lowByte;
    return result;
}
