#include "sensor.h"
#include <JY901.h>

class JY901Sensor : public Sensor
{
public:
    JY901Sensor() = default;
    ~JY901Sensor() override = default;
    void motionSetup() override final;
    void motionLoop() override final;
    void sendData() override final;
    void startCalibration(int calibrationType) override final;
    void setupJY901(uint8_t sensorId = 0, uint8_t addr = 0x50);
private:
    JY901 imu{JY901(0x50)};
    uint8_t addr = 0x50;
    uint8_t tap;
    float a[3];
};