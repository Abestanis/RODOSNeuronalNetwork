//
// Created by Sebastian Scholz on 28.03.2020.
//

#pragma once

#include "rodos.h"

enum LSM9DS1Address : uint8_t {
    DEVICE_ID = 0x0F,
    GYRO_CONTROL1 = 0x10,
    ACCEL_CONTROL5 = 0x1F,
    ACCEL_CONTROL6 = 0x20,
    MAG_CONTROL2 = 0x21,
    SENSOR_CONTROL = 0x22,
    STATUS = 0x27,
    ACCEL_X_OUT = 0x28,
    ACCEL_Y_OUT = 0x2A,
    ACCEL_Z_OUT = 0x2C,
    MAG_X_OUT = 0x28,
    MAG_Y_OUT = 0x2A,
    MAG_Z_OUT = 0x2C,
    GYRO_X_OUT = 0x18,
    GYRO_Y_OUT = 0x1A,
    GYRO_Z_OUT = 0x1C,
    TEMPERATURE_OUT = 0x15,
};

class LSM9DS1 {
protected:
    class ChipSelector {
    public:
        explicit ChipSelector(HAL_GPIO &chipSelect) : chipSelect(&chipSelect) {
            chipSelect.setPins(0);
        }
        
        ~ChipSelector() {
            chipSelect->setPins(1);
        }
    
    private:
        HAL_GPIO* chipSelect;
    };

public:
    typedef struct {
        int16_t x;
        int16_t y;
        int16_t z;
    } Int16Vec3d;
    
    explicit LSM9DS1(SPI_IDX spiId = SPI_IDX1, GPIO_PIN spiClockPin = GPIO_019,
                     GPIO_PIN spiMisoPin = GPIO_020, GPIO_PIN spiMosiPin = GPIO_021,
                     GPIO_PIN accGyroSelectPin = GPIO_006, GPIO_PIN magSelectPin = GPIO_041);
    
    int32_t init();
    
    int32_t enableAccelerometer();
    
    int32_t enableGyroscope();
    
    int32_t enableMagnetometer();
    
    uint8_t getChipId(HAL_GPIO &chipSelect);
    
    uint8_t getStatus(HAL_GPIO &chipSelect);
    
    int32_t readAccelerometer(Int16Vec3d* data);
    
    int32_t readGyroscope(Int16Vec3d* data);
    
    int32_t readMagnetometer(Int16Vec3d* data);
    
    int32_t readTemperature(int16_t* temperature);
    
    //private:
    
    void waitForStatus(HAL_GPIO &chipSelect, uint8_t statusFlag);
    
    int32_t read(HAL_GPIO &chipSelect, LSM9DS1Address address, uint8_t* buffer, size_t bufferSize);
    
    uint8_t readByte(HAL_GPIO &chipSelect, LSM9DS1Address address);
    
    int32_t write(HAL_GPIO &chipSelect, LSM9DS1Address address, uint8_t* buffer, size_t bufferSize);
    
    int32_t writeByte(HAL_GPIO &chipSelect, LSM9DS1Address address, uint8_t byte);
    
    static int16_t bytesToInt16(const uint8_t* data);

private:
    HAL_SPI spi;
    HAL_GPIO chipSelectAccelGyro;
    HAL_GPIO chipSelectMag;
};
