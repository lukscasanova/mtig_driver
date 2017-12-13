#define THROTTLE_VALUE 10
#define FRAME_ID_STRING "xsens"

#include <xsensdeviceapi.h>

#include <ros/ros.h>
#include <ros/console.h>

#define PI 3.14159265359
#define GRAVITY 9.80665

/**
 * @struct outputSettings
 * @brief hold settings about the activation and operating frequency of each module
 */
typedef struct _outputSettings{
			int orientationFreq;
			bool orientationData;

			int gpsFreq;
			bool gpsData;

			int temperatureFreq;
			bool temperatureData;

			int accelerationFreq;
			bool accelerationData;

			int pressureFreq;
			bool pressureData;

			int magneticFreq;
			bool magneticData;

			int altitudeFreq;
			bool altitudeData;

			int gyroscopeFreq;
			bool gyroscopeData;

			int velocityFreq;
			bool velocityData;
		} outputSettings;



