#include "balance_mpc/mpu6050_linux.h" // Adjusted include path
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <cstdio> // For perror, fprintf, stdout
#include <cstdlib> // For exit, EXIT_FAILURE if needed

extern "C" {
#include <linux/i2c-dev.h>
}

MPU6050::MPU6050(const std::string& i2c_bus_path, uint8_t address)
    : bus_path_(i2c_bus_path), i2c_address_(address), fd_(-1) {}

MPU6050::~MPU6050() {
    if (fd_ >= 0) {
        close(fd_);
    }
}

bool MPU6050::init() {
    fd_ = open(bus_path_.c_str(), O_RDWR);
    if (fd_ < 0) {
        perror("Failed to open I2C bus");
        return false;
    }

    if (ioctl(fd_, I2C_SLAVE, i2c_address_) < 0) {
        perror("Failed to acquire bus access and/or talk to slave");
        close(fd_);
        fd_ = -1;
        return false;
    }

    // Wake up MPU6050 (clear sleep bit in PWR_MGMT_1)
    if (!writeRegister(MPU6050_PWR_MGMT_1, 0x00)) {
        fprintf(stderr, "Failed to wake up MPU6050\n");
        return false;
    }
    usleep(100000); // Wait for sensor to stabilize

    // Set accelerometer and gyro ranges (optional, add more config as needed)
    // Example: Set accelerometer to +/- 2g
    if (!writeRegister(MPU6050_ACCEL_CONFIG, MPU6050_ACCEL_FS_2G)) {
         fprintf(stderr, "Failed to set accel config\n");
         // return false; // Decide if this is critical
    }
    // Example: Set gyro to +/- 250 dps
    if (!writeRegister(MPU6050_GYRO_CONFIG, MPU6050_GYRO_FS_250DPS)) {
         fprintf(stderr, "Failed to set gyro config\n");
         // return false; // Decide if this is critical
    }
    
    return testConnection();
}

bool MPU6050::testConnection() {
    if (fd_ < 0) return false;
    uint8_t who_am_i_val = 0;
    if (readRegisters(MPU6050_WHO_AM_I, 1, &who_am_i_val)) {
        // MPU6050 WHO_AM_I default value is 0x68.
        // It can also be 0x98, 0x72 etc depending on chip version and if AD0 is high/low
        if (who_am_i_val == 0x68 || who_am_i_val == 0x98 || who_am_i_val == 0x72 || who_am_i_val == 0x70 ) { // common values
             fprintf(stdout, "MPU6050 connection successful (WHO_AM_I: 0x%X)\n", who_am_i_val);
            return true;
        } else {
            fprintf(stderr, "MPU6050 WHO_AM_I check failed (Value: 0x%X). Expected 0x68 or similar.\n", who_am_i_val);
        }
    } else {
        fprintf(stderr, "Failed to read WHO_AM_I register.\n");
    }
    return false;
}


bool MPU6050::writeRegister(uint8_t reg, uint8_t value) {
    if (fd_ < 0) return false;
    uint8_t buffer[2] = {reg, value};
    if (::write(fd_, buffer, 2) != 2) {
        // perror("Failed to write to I2C register"); // Commented out for ROS_ERROR usage later
        return false;
    }
    return true;
}

bool MPU6050::readRegisters(uint8_t reg, uint8_t count, uint8_t* dest) {
    if (fd_ < 0) return false;
    // Write the register address to start reading from
    if (::write(fd_, &reg, 1) != 1) {
        // perror("Failed to write register address for reading");
        return false;
    }
    // Read the data
    if (::read(fd_, dest, count) != count) {
        // perror("Failed to read from I2C registers");
        return false;
    }
    return true;
}

bool MPU6050::readRawData(int16_t& ax, int16_t& ay, int16_t& az, int16_t& gx, int16_t& gy, int16_t& gz) {
    if (fd_ < 0) return false;
    uint8_t buffer[14]; // 6 for accel, (skip temp), 6 for gyro

    // Read accelerometer data (6 bytes starting from ACCEL_XOUT_H)
    if (!readRegisters(MPU6050_ACCEL_XOUT_H, 6, buffer)) {
        return false;
    }
    ax = (int16_t)((buffer[0] << 8) | buffer[1]);
    ay = (int16_t)((buffer[2] << 8) | buffer[3]);
    az = (int16_t)((buffer[4] << 8) | buffer[5]);

    // Read gyroscope data (6 bytes starting from GYRO_XOUT_H)
    if (!readRegisters(MPU6050_GYRO_XOUT_H, 6, buffer)) { // Re-use buffer for gyro
        return false;
    }
    gx = (int16_t)((buffer[0] << 8) | buffer[1]);
    gy = (int16_t)((buffer[2] << 8) | buffer[3]);
    gz = (int16_t)((buffer[4] << 8) | buffer[5]);
    
    return true;
}

bool MPU6050::getMotionData(MPU6050Data& data) {
    int16_t ax_raw, ay_raw, az_raw;
    int16_t gx_raw, gy_raw, gz_raw;

    if (!readRawData(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw)) {
        return false;
    }

    data.accel_x = (float)ax_raw / accel_lsb_sensitivity_;
    data.accel_y = (float)ay_raw / accel_lsb_sensitivity_;
    data.accel_z = (float)az_raw / accel_lsb_sensitivity_;

    data.gyro_x = (float)gx_raw / gyro_lsb_sensitivity_;
    data.gyro_y = (float)gy_raw / gyro_lsb_sensitivity_;
    data.gyro_z = (float)gz_raw / gyro_lsb_sensitivity_;
    
    return true;
} 