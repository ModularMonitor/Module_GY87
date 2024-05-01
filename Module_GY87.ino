#include "mcombo.h"
#include "Serial/packaging.h"
#include "Serial/flags.h"

using namespace CS;

PackagedWired* wire;
mGY87* mcb = nullptr;
const auto this_device = device_id::GY87_SENSOR;

void callback(void*, const uint8_t, const char*, const uint8_t);

void setup() {
    Serial.begin(115200);
    
    mcb = new mGY87();
    
    wire = new PackagedWired(config()
        .set_slave(this_device)
        .set_slave_callback(callback)
        .set_led(2)
    );
}

void callback(void* rw, const uint8_t expects, const char* received, const uint8_t length)
{
    if (length != sizeof(Requester)) return;
    
    PackagedWired& w = *(PackagedWired*) rw;
    Requester req(received);
    
    switch(req.get_offset()) {
    case 0:
    {
        FlagWrapper fw;
        if (mcb->has_issues())              fw |= device_flags::HAS_ISSUES;
        if (mcb->has_new_data_autoreset())  fw |= device_flags::HAS_NEW_DATA;
        
        Command cmd("#FLAGS", (uint64_t)fw);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 1:
    {
        const float val = mcb->get_temperature();
        Command cmd("/gy87/bmp085/temperature", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 2:
    {
        const int64_t val = mcb->get_pressure_pa();
        Command cmd("/gy87/bmp085/pressure", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 3:
    {
        const float val = mcb->get_altitude();
        Command cmd("/gy87/bmp085/altitude", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 4:
    {
        const int64_t val = mcb->get_pressure_seal_level_pa();
        Command cmd("/gy87/bmp085/pressure_sea_level", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 5:
    {
        const float val = mcb->get_real_altitude();
        Command cmd("/gy87/bmp085/altitude_real", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 6:
    {
        const int64_t val = mcb->get_accel_x_raw();
        Command cmd("/gy87/mpu6050/accel/x/raw", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 7:
    {
        const int64_t val = mcb->get_accel_y_raw();
        Command cmd("/gy87/mpu6050/accel/y/raw", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 8:
    {
        const int64_t val = mcb->get_accel_z_raw();
        Command cmd("/gy87/mpu6050/accel/z/raw", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 9:
    {
        const int64_t val = mcb->get_gyro_x_raw();
        Command cmd("/gy87/mpu6050/gyro/x/raw", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 10:
    {
        const int64_t val = mcb->get_gyro_y_raw();
        Command cmd("/gy87/mpu6050/gyro/y/raw", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 11:
    {
        const int64_t val = mcb->get_gyro_z_raw();
        Command cmd("/gy87/mpu6050/gyro/z/raw", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 12:
    {
        const int64_t val = mcb->get_yaw();
        Command cmd("/gy87/mpu6050/yaw", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 13:
    {
        const int64_t val = mcb->get_pitch();
        Command cmd("/gy87/mpu6050/pitch", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 14:
    {
        const int64_t val = mcb->get_roll();
        Command cmd("/gy87/mpu6050/roll", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 15:
    {
        const int64_t val = mcb->get_accel_x();
        Command cmd("/gy87/mpu6050/accel/x", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 16:
    {
        const int64_t val = mcb->get_accel_y();
        Command cmd("/gy87/mpu6050/accel/y", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 17:
    {
        const int64_t val = mcb->get_accel_z();
        Command cmd("/gy87/mpu6050/accel/z", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    /*case 18:
    {
        const float val = mcb->get_heading();
        Command cmd("/gy87/hmc58831/heading", val);
        w.slave_reply_from_callback(cmd);
    }
    break;*/
    default:
    {
        Command cmd; // invalid
        w.slave_reply_from_callback(cmd);
    }
    }
}

// unused
void loop() { vTaskDelete(NULL); }