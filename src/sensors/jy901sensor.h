#ifndef SENSORS_JY901SENSOR_H
#define SENSORS_JY901SENSOR_H

#include "sensor.h"
#include <JY901.h>

class JY901Sensor : public Sensor
{
public:
	JY901Sensor(
		uint8_t id,
		uint8_t address,
		float rotation,
		uint8_t sclPin,
		uint8_t sdaPin
	)
		: Sensor("JY901Sensor", IMU_JY901, id, address, rotation, sclPin, sdaPin){};
	~JY901Sensor() override = default;
    void motionSetup() override final;
    void motionLoop() override final;
    void sendData() override final;
    void startCalibration(int calibrationType) override final;
private:
    JY901 imu{JY901(0x50)};
    uint8_t tap;
};
#endif