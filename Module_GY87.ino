#include "mcombo.h"
#include "Serial/packaging.h"

using namespace CS;

PackagedWired* wire;
mGY87* mcb = nullptr;
const auto this_device = device_id::GY87_SENSOR;

void callback(void*, const uint8_t, const char*, const uint8_t);

void setup() {
    Serial.begin(115200);
    while(!Serial);

    Serial.printf("Starting SLAVE\n");
    
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
        const float val = mcb->get_temperature();
        Command cmd("/gy87/bmp085/temperature", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 1:
    {
        const int64_t val = mcb->get_pressure_pa();
        Command cmd("/gy87/bmp085/pressure", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 2:
    {
        const float val = mcb->get_altitude();
        Command cmd("/gy87/bmp085/altitude", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 3:
    {
        const int64_t val = mcb->get_pressure_seal_level_pa();
        Command cmd("/gy87/bmp085/pressure_sea_level", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 4:
    {
        const float val = mcb->get_real_altitude();
        Command cmd("/gy87/bmp085/altitude_real", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 5:
    {
        const float val = mcb->get_heading();
        Command cmd("/gy87/hmc58831/heading", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 6:
    {
        const int64_t val = mcb->get_accel_x();
        Command cmd("/gy87/mpu6050/accel/x", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 7:
    {
        const int64_t val = mcb->get_accel_y();
        Command cmd("/gy87/mpu6050/accel/y", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 8:
    {
        const int64_t val = mcb->get_accel_z();
        Command cmd("/gy87/mpu6050/accel/z", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 9:
    {
        const int64_t val = mcb->get_gyro_x();
        Command cmd("/gy87/mpu6050/gyro/x", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 10:
    {
        const int64_t val = mcb->get_gyro_y();
        Command cmd("/gy87/mpu6050/gyro/y", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    case 11:
    {
        const int64_t val = mcb->get_gyro_z();
        Command cmd("/gy87/mpu6050/gyro/z", val);
        w.slave_reply_from_callback(cmd);
    }
    break;
    default:
    {
        Command cmd; // invalid
        w.slave_reply_from_callback(cmd);
    }
    }
}

// unused
void loop() { vTaskDelete(NULL); }