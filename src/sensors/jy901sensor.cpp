/*
    MIT License

    Copyright (c) 2021 Sark1tama

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#include "sensors/jy901sensor.h"
#include "GlobalVars.h"
#include "utils.h"


void JY901Sensor::motionSetup()
{
	if (!imu.StartIIC(addr))
    {
		m_Logger.fatal(
			"Can't connect to %s at address 0x%02x",
			getIMUNameByType(sensorType),
			addr
		);
		ledManager.pattern(50, 50, 200);
		return;
    }
	m_Logger.info(
		"Connected to %s on 0x%02x. "
		"Info: SW Version Major: 0x%02x ",
		getIMUNameByType(sensorType),
		addr
	);
	working = true;
}

void JY901Sensor::motionLoop()
{
    imu.GetQuater();
    
	fusedRotation.set(
		(float)imu.stcQuater.q1 / 32768,
		(float)imu.stcQuater.q2 / 32768,
		(float)imu.stcQuater.q3 / 32768,
		(float)imu.stcQuater.q0 / 32768
	);
	fusedRotation *= sensorOffset;
#if SEND_ACCELERATION
	{
		imu.GetAcc();
		acceleration[0] = (float)imu.stcAcc.a[0]/32768*16;
		acceleration[1] = (float)imu.stcAcc.a[1]/32768*16;
		acceleration[2] = (float)imu.stcAcc.a[2]/32768*16;
		setAccelerationReady();
	}
#endif  // SEND_ACCELERATION
	setFusedRotationReady();
	tap = imu.GetTapDetector();
}

void JY901Sensor::sendData()
{
	if (newFusedRotation) {
		newFusedRotation = false;
		networkConnection.sendRotationData(
			sensorId,
			&fusedRotation,
			DATA_TYPE_NORMAL,
			calibrationAccuracy
		);

#ifdef DEBUG_SENSOR
		m_Logger.trace("Quaternion: %f, %f, %f, %f", UNPACK_QUATERNION(fusedRotation));
#endif

#if SEND_ACCELERATION
		if (newAcceleration) {
			newAcceleration = false;
			networkConnection.sendSensorAcceleration(
				this->sensorId,
				this->acceleration
			);
		}
#endif
	}
	if (tap != 0) {
		networkConnection.sendSensorTap(sensorId, tap);
		tap = 0;
	}
}

void JY901Sensor::startCalibration(int calibrationType)
{
#ifdef CALIBRATING_LED
	calibreledManager.pattern(20, 20, 10);
	calibreledManager.blink(2000);
#endif
	imu.Unlock();
    delay(2000);
#ifdef CALIBRATING_LED
	calibreledManager.on();
#endif
	imu.SetDirection(1);
    imu.SetCalsw(calibrationType);
    delay(6000);
#ifdef CALIBRATING_LED
	calibreledManager.off();
#endif
	imu.Save(0);
    delay(100);
}