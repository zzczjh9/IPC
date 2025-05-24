#ifndef MPU6050_LINUX_H
#define MPU6050_LINUX_H

#include <string>
#include <cstdint>

// MPU6050 Register Defines
#define MPU6050_ADDR            0x68
#define MPU6050_PWR_MGMT_1      0x6B
#define MPU6050_ACCEL_XOUT_H    0x3B
#define MPU6050_GYRO_XOUT_H     0x43
#define MPU6050_WHO_AM_I        0x75

// Configuration options (simplified, add more as needed)
#define MPU6050_ACCEL_FS_2G     0x00
#define MPU6050_GYRO_FS_250DPS  0x00
#define MPU6050_ACCEL_CONFIG    0x1C
#define MPU6050_GYRO_CONFIG     0x1B


struct MPU6050Data {
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
    // Add temperature if needed
};

class MPU6050 {
public:
    MPU6050(const std::string& i2c_bus_path = "/dev/i2c-2", uint8_t address = MPU6050_ADDR);
    ~MPU6050();

    bool init();
    bool testConnection(); // Check WHO_AM_I register
    bool readRawData(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz);
    bool getMotionData(MPU6050Data& data);

    // Basic register read/write
    bool writeRegister(uint8_t reg, uint8_t value);
    bool readRegisters(uint8_t reg, uint8_t count, uint8_t* dest);

private:
    std::string bus_path_;
    uint8_t i2c_address_;
    int fd_; // File descriptor for the I2C bus

    // Conversion factors (example for +/- 2g and +/- 250 dps)
    // These depend on the full-scale range set.
    const float accel_lsb_sensitivity_ = 16384.0f; // For FS_SEL=0 (+/- 2g)
    const float gyro_lsb_sensitivity_ = 131.0f;   // For FS_SEL=0 (+/- 250 dps)
};

#endif // MPU6050_LINUX_H 