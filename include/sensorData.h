/**
 * @class SensorData
 * @brief Stores different data types coming in packets (XsDataPackets) from the sensors in the device.
 * @details This class contains one important function that takes an data packet from the device
 *          and inspect it for all the data from all the modules enable in outputSettings. It stores this
 *          data and provides getters for it.
 * @author Lucas Casanova Nogueira (lucas_afonso@hotmail.com)
 * @date 2014
 */
#ifndef SENSOR_DATA
#define SENSOR_DATA
#include "global.h"
#include <tf/tf.h>

//[Manual] http://amtechs.co.jp/2_gps/pdf/MTi%20User%20Manual.pdf

class SensorData{
	//Mti-G Data
private:
	static const unsigned char GPS_FIX   = 0x04; 
	float accX, accY, accZ;
	float acc_error;
	float gyrX, gyrY, gyrZ;
	float gyr_error; // covariance matrix associated with angular velocity of IMU, error from [Manual], page 11
    	float magX, magY, magZ;
    	float mTemperature, mPressure;
    	float q0, q1, q2, q3;
    	float eroll, epitch, eyaw;
	float roll_error, pitch_error, yaw_error;
	float acc_noise, gyr_noise;
    	unsigned int ts;
	tf::Quaternion q_orientation;

	
	float m_hdop, m_vdop, m_gdop, m_pdop, m_tdop, m_ndop, m_edop, m_itow;
	float mPositionAccuracy, mSpeedAccuracy;
	int mSatelliteNumber, mGpsFixStatus;

	float mAltitude, mLongitude, mLatitude;
	float mVelocityNorth, mVelocityEast, mVelocityDown;
	float mVelocityX, mVelocityY, mVelocityZ;
	unsigned char mStatus;
	uint32_t mHorizontalAccuracy, mVerticalAccuracy;
	std::string frameId_string;

public:
	//void setSettings(outputSettings &mSettings){ this->

	float hdop(){ return m_hdop;}
	float vdop(){ return m_vdop;}
	float gdop(){ return m_gdop;}
	float pdop(){ return m_pdop;}
	float tdop(){ return m_tdop;}
	float ndop(){ return m_ndop;}
	float edop(){ return m_edop;}
	float itow(){ return m_itow;}
	

	float accError() {return acc_error;}
	float gyrError() {return gyr_error;}
	float rollError() {return roll_error;}
	float pitchError() {return pitch_error;}
	float yawError() {return yaw_error;}
	float to_rad_sqr(float x){return (x*x*PI*PI/(180*180));}


	float calculateAccErrorSquared(float noise_density, float freq);
	float calculateGyrErrorSquared(float noise_density, float freq);

	void getOrientationQuaternion(tf::Quaternion * q){
	    *q = tf::Quaternion(q1, q2, q3, q0);
	}

	float PositionAccuracy() { return mPositionAccuracy; }
	float SpeedAccuracy() { return mSpeedAccuracy; }
	
	int SatelliteNumber() { return mSatelliteNumber; }
	int GpsFixStatus() { return mGpsFixStatus;}
	  
	float accelerometer_x() { return accX; }
	float accelerometer_y() { return accY; }
	float accelerometer_z() { return accZ; }
	//frame local
	float gyroscope_x() { return gyrX; }
	float gyroscope_y() { return gyrY; }
	float gyroscope_z() { return gyrZ; }
	
	float magnetic_x() { return magX; } 
	float magnetic_y() { return magY; }
	float magnetic_z() { return magZ; }
	
	float temperature() { return mTemperature; }
	float pressure() { return mPressure; }

	float quaternion_x() { return q1; }
	float quaternion_y() { return q2; }
	float quaternion_z() { return q3; }
	float quaternion_w() { return q0; }

	float roll() { return eroll; }
	float pitch() { return epitch; }
	float yaw() { return eyaw; }

	float altitude() { return mAltitude; }
	float longitude() { return mLongitude; }
	float latitude() { return mLatitude; }
	
	//frame local
	float velocity_x() { return mVelocityX; }
	float velocity_y() { return mVelocityY; }
	float velocity_z() { return mVelocityZ; }
		
	float velocityNorth() { return mVelocityNorth; }
	float velocityEast() { return mVelocityEast; }
	float velocityDown() { return mVelocityDown; }

	bool GPSFix() { return mGpsFixStatus; } 

	uint32_t horizontalAccuracy() { return mHorizontalAccuracy; }
	uint32_t verticalAccuracy() { return mVerticalAccuracy; }
		
	std::string 	frameId() { return frameId_string; }

	SensorData(outputSettings &mSettings);
	SensorData();

	void fillData(XsDataPacket *);

};

#endif
