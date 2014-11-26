/**
 * @file sensorData.cpp
 * @brief Stores different data types coming in packets (XsDataPackets) from the sensors in the device.
 * @details This class contains one important function that takes an data packet from the device
 *          and inspect it for all the data from all the modules enable in outputSettings. It stores this
 *          data and provides getters for it.
 * @author Lucas Casanova Nogueira (lucas_afonso@hotmail.com)
 * @date 2014
 */

#include "sensorData.h"

/**
 * @brief Initializes all data with blank values 
 */
SensorData::SensorData(	outputSettings  & mSettings){
  
	accX = accY = accZ = 0.0;
	gyrX = gyrY = gyrZ = 0.0;
	magX = magY = magZ = 0.0;
	q0 = q1 = q2 = q3 = 0.0;
	eroll = epitch = eyaw = 0.0;
	mTemperature = mPressure = 0.0;
	mVelocityX = mVelocityY = mVelocityZ = 0.0;
	mStatus = 0;
	mHorizontalAccuracy = mVerticalAccuracy = 0;
	mAltitude = mLongitude = mLatitude = 0.0;
	mVelocityNorth = mVelocityEast = mVelocityDown = 0.0;
	
	

	//Get measurements errors from the launchfile. They should be already ^2 because they'll go straight to the covariance matrix
	if(ros::param::get("~roll_error", roll_error)){
		roll_error=to_rad_sqr(roll_error);		
	}
	else{
		ROS_ERROR("No roll_error value defined in launchfile");	
	}

	if(ros::param::get("~pitch_error", pitch_error)){
		pitch_error=to_rad_sqr(pitch_error);		
	}
	else{
		ROS_ERROR("No pitch_error value defined in launchfile");	
	}

	if(ros::param::get("~yaw_error", yaw_error)){
		yaw_error=to_rad_sqr(yaw_error);		
	}
	else{
		ROS_ERROR("No yaw_error value defined in launchfile");	
	}

	if(!ros::param::get("~acc_noise", acc_noise)){
		ROS_ERROR("No acc_error value defined in launchfile");	
	}
	else{
		acc_error=calculateAccErrorSquared(acc_noise, mSettings.accelerationFreq);
	}	

	if(!ros::param::get("~gyr_noise", gyr_noise)){
		ROS_ERROR("No noise_density value defined in launchfile");
	}
	else{
		gyr_error= calculateGyrErrorSquared(gyr_noise, mSettings.gyroscopeFreq);
	}


	ROS_DEBUG_STREAM("GYR_ERROR=" << gyr_error);
	ros::param::param<std::string>("~frame_id", frameId_string, "xsens");
}
SensorData::SensorData(){

}


/**
 *	\brief Obtains the Acceleration Error ^2 from the noise density and sensor frequency
 *      \param noise_density in [g/sqrt(Hz)], 
 *      \param freq  in [Hz],
 *      \return error in (m/s^2)^2
 */
float SensorData::calculateAccErrorSquared(float noise_density, float freq){
	return (noise_density*GRAVITY)*(noise_density*GRAVITY)*freq;

}


/**
 *      \brief Obtains the Gyroscope Error ^2 from the noise density and the sensor frequency
 *      \param noise_density in [g/sqrt(Hz)] 
 *      \param freq  in [Hz]
 *      \return error in (m/s^2)^2
 */
float SensorData::calculateGyrErrorSquared(float noise_density, float freq){
	return to_rad_sqr(noise_density)*freq;
}

/**
 * \brief Fills data using XsDataPacket object 
 * @param _packet Incoming packet from Xsens device.
 */
void SensorData::fillData(XsDataPacket * _packet){
		XsDataPacket packet = *_packet;
		XsMessage msg = packet.originalMessage();
		XsSize msg_size = msg.getDataSize();

		ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "Data Size: " << msg_size);

		
	// Get the orientation data
		if (packet.containsOrientation()) {
			XsQuaternion quaternion = packet.orientationQuaternion();

 			q1 = quaternion.m_x;
			q2 = quaternion.m_y;
			q3 = quaternion.m_z;
			q0 = quaternion.m_w;
						
			XsEuler euler = packet.orientationEuler();
			eroll = euler.m_roll;
			epitch = euler.m_pitch;
			eyaw = euler.m_yaw;
 			
			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE,"Orientation: Roll:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_roll
							  << ", Pitch:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_pitch
							  << ", Yaw:" << std::setw(7) << std::fixed << std::setprecision(2) << euler.m_yaw);
						
		}

		// Get the gyroscope data
		if (packet.containsCalibratedGyroscopeData()) {
			XsVector gyroscope = packet.calibratedGyroscopeData();
		
			gyrX = gyroscope.at(0);
			gyrY = gyroscope.at(1);
			gyrZ = gyroscope.at(2);

			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "Angular Velocity: ( "
							 << gyrX << ", " << gyrY << ", " << gyrZ << ")" ); 
		}

		// Get the acceleration data
		if (packet.containsCalibratedAcceleration()) {
			XsVector acceleration = packet.calibratedAcceleration();

			accX = acceleration.at(0);
			accY = acceleration.at(1);
			accZ = acceleration.at(2);
			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "Linear Acceleration: (" 
							<< accX << "," << accX << "," << accZ << ")" );
		}

		//Get the altitude data
		if(packet.containsAltitude()){
			mAltitude = packet.altitude();
			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "containsAltitude: ");
		}

			
		//Get the Latitude and Longitude Data
		if(packet.containsLatitudeLongitude()){
			XsVector latlon = packet.latitudeLongitude();
			mLatitude = latlon[0];
			mLongitude = latlon[1];
			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "containsLatitudeLongitude");
		}

		// Get the magnetic field data
		if (packet.containsCalibratedMagneticField()) {
			XsVector magneticField = packet.calibratedMagneticField();

			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "Magnetic Field: ("<< magneticField[0] <<" , " 
							<< magneticField[1] << " , "<< magneticField[2]   << ")");	

			magX = magneticField.at(0);
			magY = magneticField.at(1);
			magZ = magneticField.at(2);
		}
					
		// Get Temperature data
		if (packet.containsTemperature()){
			double temperature = packet.temperature();
			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "Temperature : " << temperature);
			mTemperature = temperature;
		}

		// Get Pressure Data 
		if(packet.containsPressure() ){
			XsPressure pressure = packet.pressure();
			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "Pressure : " << pressure.m_pressure);
			mPressure= pressure.m_pressure;
		}	
	
		// Get Velocity Data
		if( packet.containsVelocity() ){
			XsVector velocity = packet.velocity();
			mVelocityX = velocity.at(0);
			mVelocityY = velocity.at(1);
			mVelocityZ = velocity.at(2);

			ROS_INFO_STREAM(" Velocity [ x (east) : " << mVelocityX << ", y (north) : "
					<< mVelocityY << ", z (up) " << mVelocityZ << " ]");
		}


		// Get GPS PVT Data
		if( packet.containsGpsPvtData() ){
			XsGpsPvtData gpsPvtData = packet.gpsPvtData();
			ROS_INFO_STREAM(" Horizontal accuracy: " << gpsPvtData.m_hacc ); 
		}

		if(packet.containsRawAcceleration() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "Raw Acceleration") ;}
		if(packet.containsRawGyroscopeData() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "Raw Gyroscope") ;}
		if(packet.containsRawGyroscopeTemperatureData() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "RawGyrTemp");}
		if(packet.containsRawMagneticField() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "RawMag");}
		if(packet.containsRawTemperature() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "RawTemp");}
		if(packet.containsRawData() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "RawData");}
		if(packet.containsCalibratedData() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "Calib Data");}
		if(packet.containsSdiData() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "SDI");}
		if(packet.containsStatusByte() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "StatusByte");}
		if(packet.containsDetailedStatus() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "DetailedStatus");}
		if(packet.containsPacketCounter8() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "PacketCounter8");}
		if(packet.containsPacketCounter() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "PacketCounter");}
		if(packet.containsSampleTimeFine() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "SampleTimeFine");}
		if(packet.containsSampleTimeCoarse() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "SampleTimeCoarse");}
		if(packet.containsSampleTime64() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "SampleTime64");}
		if(packet.containsFreeAcceleration() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "FreeAcceleration");}
		if(packet.containsPressureAge() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "PressureAge");}
		if(packet.containsAnalogIn1Data() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "AN1IN");}
		if(packet.containsAnalogIn2Data() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "AN2IN");}
		if(packet.containsPositionLLA() ){ 
			
			XsVector posLLA = packet.positionLLA();
			mLatitude = posLLA[0];
			mLongitude = posLLA[1];
			mAltitude = posLLA[2];
			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "mLatitude = " << mLatitude);
			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "mLongitude = " << mLongitude);
			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, "mAltitude = " << mAltitude);

			//for(int i=0; i < posLLA.size(); i++){
			//ROS_INFO_STREAM("posLLA[" << i << "] : " << posLLA[i] ); 
			//}
		}
		if(packet.containsUtcTime() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "UTC-Time");}
		if(packet.containsFrameRange() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "Frame Range");}
		if(packet.containsRssi() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "RSSI");}
		if(packet.containsRawGpsDop() ){ 
			//ROS_INFO_THROTTLE(THROTTLE_VALUE, "DOP");}
			XsRawGpsDop dop = packet.rawGpsDop();
			m_gdop = ((float) dop.m_gdop)/100;
			m_pdop = ((float) dop.m_pdop)/100;
			m_tdop = ((float) dop.m_tdop)/100;
			m_vdop = ((float) dop.m_vdop)/100;
			m_hdop = ((float) dop.m_hdop)/100;
			m_edop = ((float) dop.m_edop)/100;
			m_ndop = ((float) dop.m_ndop)/100;
			m_itow = ((float) dop.m_itow)/1000;

			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE, 
						"GDOP = " << m_gdop <<
						",PDOP = " << m_pdop <<						
						",TDOP = " << m_tdop <<						
						",VDOP = " << m_vdop <<						
						",HDOP = " << m_hdop <<						
						",EDOP = " << m_edop <<						
						",NDOP = " << m_ndop <<						
						",ITOW = " << m_itow);			
		}
		if(packet.containsRawGpsSol() ){
			// ROS_INFO_THROTTLE(THROTTLE_VALUE, "SOL");}
			XsRawGpsSol sol = packet.rawGpsSol();
			mPositionAccuracy = sol.m_pacc;
			mSpeedAccuracy = sol.m_sacc;
			mSatelliteNumber = sol.m_numsv;
			mGpsFixStatus = sol.m_gpsfix;
			
			ROS_INFO_STREAM_THROTTLE(THROTTLE_VALUE,
						"Pos Acc = " << mPositionAccuracy <<
						",Speed Acc = " << mSpeedAccuracy <<
						",Sat Number = " << mSatelliteNumber <<
						",GPS FIX = " << std::hex << mGpsFixStatus);
		}
		if(packet.containsRawGpsTimeUtc() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "GPS-UTC");}
		if(packet.containsRawGpsSvInfo() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "SV INFO");}
		//	if(packet.containsTriggerIndication() ){ ROS_INFO_THROTTLE(THROTTLE_VALUE, "TRIGGER");}




		//Get Status byte		
		if( packet.containsStatus() ){
			mStatus = packet.status();
			ROS_INFO_THROTTLE(10, "Status: %.8X", mStatus);
		}
}

