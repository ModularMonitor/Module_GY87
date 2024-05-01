#pragma once

#include "I2Cdev/Arduino/MPU6050/MPU6050_6Axis_MotionApps20.h"
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
    //float m_heading = 0.0f;
    // MPU6050
    int16_t m_ax = 0, m_ay = 0, m_az = 0;
    int16_t m_gx = 0, m_gy = 0, m_gz = 0;
    float m_yaw_pitch_roll[3]{0.0f};
    float m_acceleration[3]{0.0f};
    VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
    
    
    bool m_fail_flag = false;
    bool m_has_new_data = false;    

    void async() {
        MPU6050 mpu;
        Adafruit_BMP085 bmp;
        //HMC5883L_Simple Compass;
        
        Wire.begin();
        Wire.setClock(400000);
                
        while(!bmp.begin()){
            m_fail_flag = true;
            delay(200);
        }
        
        // initialize mpu6050
        mpu.initialize();
        while(!mpu.testConnection()) {
            m_fail_flag = true;
            delay(200);
        }
        //mpu.setI2CBypassEnabled(true); // set bypass mode for gateway to hmc5883L
        if (mpu.dmpInitialize() == 0) mpu.setDMPEnabled(true);

        // initialize hmc5883l
        //Compass.SetDeclination(23, 35, 'E');
        //Compass.SetSamplingMode(COMPASS_CONTINUOUS);
        //Compass.SetScale(COMPASS_SCALE_130);
        //Compass.SetOrientation(COMPASS_HORIZONTAL_X_NORTH);
        
        m_fail_flag = false;
        
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorFloat gravity;    // [x, y, z]            gravity vector
        VectorInt16 aa;         // [x, y, z]            accel sensor measurements

        uint8_t fifoBuffer[64]; // FIFO storage buffer
        uint16_t fifoCount;
        const uint16_t packetSize = mpu.dmpGetFIFOPacketSize();
        
        while(1) {            
            m_temp = bmp.readTemperature();
            m_pressure = bmp.readPressure();
            m_altitude = bmp.readAltitude();
            m_sea_level_pressure = bmp.readSealevelPressure();
            m_real_altitude = bmp.readAltitude(101500);
            
            mpu.getMotion6(&m_ax, &m_ay, &m_az, &m_gx, &m_gy, &m_gz);
            
            //m_heading = Compass.GetHeadingDegrees();
                        
            auto mpuIntStatus = mpu.getIntStatus();
            fifoCount = mpu.getFIFOCount();
            // check for overflow (this should never happen unless our code is too inefficient)
            if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                // reset so we can continue cleanly
                mpu.resetFIFO();
            }
            else if (mpuIntStatus & 0x02) {
                while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
                
                mpu.getFIFOBytes(fifoBuffer, packetSize);
                fifoCount -= packetSize;
                
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(m_yaw_pitch_roll, &q, &gravity);
                mpu.dmpGetAccel(&aa, fifoBuffer);
                mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                
                for(auto& i : m_yaw_pitch_roll) i *= 180.0f / M_PI;
            }
            
            
            m_has_new_data = true;
        }

        vTaskDelete(NULL);
    }
public:
    mGY87() : SelfThreadable("ASYNC") { async_start(); }

    float get_temperature() const { return m_temp; }
    int32_t get_pressure_pa() const { return m_pressure; }
    float get_altitude() const { return m_altitude; }
    int32_t get_pressure_seal_level_pa() const { return m_sea_level_pressure; }
    float get_real_altitude() const { return m_real_altitude; }
    //float get_heading() const { return m_heading; }
    
    int16_t get_accel_x_raw() const { return m_ax; }
    int16_t get_accel_y_raw() const { return m_ay; }
    int16_t get_accel_z_raw() const { return m_az; }
    int16_t get_gyro_x_raw() const { return m_gx; }
    int16_t get_gyro_y_raw() const { return m_gy; }
    int16_t get_gyro_z_raw() const { return m_gz; }
    
    float get_yaw() const { return m_yaw_pitch_roll[0]; }
    float get_pitch() const { return m_yaw_pitch_roll[1]; }
    float get_roll() const { return m_yaw_pitch_roll[2]; }
    
    int16_t get_accel_x() const { return aaReal.x; }
    int16_t get_accel_y() const { return aaReal.y; }
    int16_t get_accel_z() const { return aaReal.z; }
    
    bool has_issues() const { return m_fail_flag; }
    bool has_new_data_autoreset() { bool had = m_has_new_data; m_has_new_data = false; return had; }
};