#pragma once

#include "I2Cdev/Arduino/MPU6050/MPU6050.h"
#include "Adafruit-BMP085-Library/Adafruit_BMP085.h"
#include "HMC5883L_Simple/HMC5883L_Simple/HMC5883L_Simple.h"

#include "SelfThreadable/self_threadable.h"

// mGY87 wraps GY87 in a async class. This way, less things to worry about.
class mGY87 : protected SelfThreadable {
    // bmp085
    float m_temp = 0.0f; // *C
    int32_t m_pressure = 0; // Pa
    float m_altitude = 0.0f; // meters
    int32_t m_sea_level_pressure = 0; // Pa
    float m_real_altitude = 0.0f; // meters
    // Compass hmc5883l
    float m_heading = 0.0f;
    // MPU6050
    int16_t m_ax = 0, m_ay = 0, m_az = 0;
    int16_t m_gx = 0, m_gy = 0, m_gz = 0;
    
    bool m_fail_flag = true;
    

    void async() {
        MPU6050 accelgyro;
        Adafruit_BMP085 bmp;
        HMC5883L_Simple Compass;
        
        Wire.begin();
                
        while(!bmp.begin()){
            m_fail_flag = true;
            delay(200);
        }
        
        // initialize mpu6050
        accelgyro.initialize();
        while(!accelgyro.testConnection()) {
            m_fail_flag = true;
            delay(200);
        }
        accelgyro.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883L

        // initialize hmc5883l
        Compass.SetDeclination(23, 35, 'E');
        Compass.SetSamplingMode(COMPASS_SINGLE);
        Compass.SetScale(COMPASS_SCALE_130);
        Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
        
        m_fail_flag = false;
        while(1) {
            delay(1000);
            
            m_temp = bmp.readTemperature();
            m_pressure = bmp.readPressure();
            m_altitude = bmp.readAltitude();
            m_sea_level_pressure = bmp.readSealevelPressure();
            m_real_altitude = bmp.readAltitude(101500);
            
            accelgyro.getMotion6(&m_ax, &m_ay, &m_az, &m_gx, &m_gy, &m_gz);
            
            m_heading = Compass.GetHeadingDegrees();
        }

        vTaskDelete(NULL);
    }
public:
    mGY87() : SelfThreadable("ASYNC_CCS") { async_start(); }

    float get_temperature() const { return m_temp; }
    int32_t get_pressure_pa() const { return m_pressure; }
    float get_altitude() const { return m_altitude; }
    int32_t get_pressure_seal_level_pa() const { return m_sea_level_pressure; }
    float get_real_altitude() const { return m_real_altitude; }
    float get_heading() const { return m_heading; }
    
    int16_t get_accel_x() const { return m_ax; }
    int16_t get_accel_y() const { return m_ay; }
    int16_t get_accel_z() const { return m_az; }
    int16_t get_gyro_x() const { return m_gx; }
    int16_t get_gyro_y() const { return m_gy; }
    int16_t get_gyro_z() const { return m_gz; }
    
    bool has_issues() const;
};

