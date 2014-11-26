/**
 * @file mtiG.cpp
 * @brief ROS driver for Xsens mti-G-700.
 * 
 * @details This class is the main part of a driver for Xsens mti-G-700.
 * 	It reads packets from the device using the Xsens API - see disclosure below.
 * 	It stores the data in another class, named SensorData and
 * 	uses MessageMaker to publish this data in ROS topics.
 * 
 * @author Lucas Casanova Nogueira (lucas_afonso@hotmail.com)
 * @date 2014
 */

/* Copyright (c) Xsens Technologies B.V., 2006-2012. All rights reserved.

	This source code is provided under the MT SDK Software License Agreement
	and is intended for use only by Xsens Technologies BV and
	those that have explicit written permission to use it from
	Xsens Technologies BV.

	THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
	KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
	IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
	PARTICULAR PURPOSE.
 */


#include "mtiG.h"
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
/**
 * @brief Initializes data values and configures the sensor with desired modules
 * @param _device XsDevice from Xsens API
 */
mtiG::mtiG(XsDevice * _device, int argc, char ** argv):device(_device){

	ros::param::get("~override", override);
	parseOptions(argc, argv);


	if(override){
		//configures Xsensor device with mSettings
		ROS_DEBUG("OVERRIDE MODE");
		configure();
	}else{
		// uses the current device settings - to be used in conjunction with Windows GUI tool
		ROS_DEBUG("UNALTERED MODE");
	}
	readSettings();
	printSettings();
	sensorData=SensorData(mSettings);
	messageMaker = new MessageMaker(sensorData);
	// Advertise all messages published in this node, uses mSettings
	advertise();
}

/**
 * @brief Simplified constructor.
 * @param _device - XsDevice from Xsens API
 */
mtiG::mtiG(XsDevice * _device){
	mtiG(_device, 1, NULL);
}


/**
 * @brief Parses the parameters from the launchfile.
 * @details 
 * - Enables individual data packets.
 * - Sets the frequency of each module.
 */
void mtiG::parseOptions(int argc, char** argv){


	ros::param::get("~orientation_enabled", mSettings.orientationData);
	ros::param::get("~gps_enabled", mSettings.gpsData);
	ros::param::get("~temperature_enabled", mSettings.temperatureData);
	ros::param::get("~acceleration_enabled", mSettings.accelerationData);
	ros::param::get("~pressure_enabled", mSettings.pressureData);
	ros::param::get("~magnetic_enabled", mSettings.magneticData);
	ros::param::get("~altitude_enabled", mSettings.altitudeData);
	ros::param::get("~gyroscope_enabled", mSettings.gyroscopeData);
	ros::param::get("~velocity_enabled", mSettings.velocityData);

	ros::param::get("~orientation_frequency", mSettings.orientationFreq);
	ros::param::get("~gps_frequency", mSettings.gpsFreq);
	ros::param::get("~temperature_frequency", mSettings.temperatureFreq);
	ros::param::get("~acceleration_frequency", mSettings.accelerationFreq);
	ros::param::get("~pressure_frequency", mSettings.pressureFreq);
	ros::param::get("~magnetic_frequency", mSettings.magneticFreq);
	ros::param::get("~altitude_frequency", mSettings.altitudeFreq);
	ros::param::get("~gyroscope_frequency", mSettings.gyroscopeFreq);
	ros::param::get("~velocity_frequency", mSettings.velocityFreq);
}

/**
 *  @brief Prints the active settings.
 *  @details Reads the settings currently stored in the device
 *  and prints it in ROS debug stream.
 */
void mtiG::printSettings(){
	XsOutputConfigurationArray deviceConfig = device->outputConfiguration();

	//Print settings from the device
	for(int i =0; i < deviceConfig.size() ; i++)
	{
 			ROS_DEBUG("%.8x; Freq = %d", deviceConfig[i].m_dataIdentifier, deviceConfig[i].m_frequency);
	}
	
	//Print the settings stored in mSettings
	ROS_DEBUG_STREAM(
	"\nOrientation = " << mSettings.orientationData << 
	"\nGPS = " << mSettings.gpsData <<
	"\nTemperature = " << mSettings.temperatureData <<
	"\nAcceleration = " << mSettings.accelerationData <<
   "\nPressure = " << 	mSettings.pressureData <<
	"\nMagnetic = " << mSettings.magneticData <<
	"\nAltitude = " << mSettings.altitudeData <<
	"\nGyroscope = " << mSettings.gyroscopeData <<
	"\nVelocity = " << mSettings.velocityData <<
	"\nOrientation Frequency:" << mSettings.orientationFreq <<
	"\nGPS Frequency:" << mSettings.gpsFreq <<
	"\nTemperature Frequency:" << mSettings.temperatureFreq <<
	"\nAcceleration Frequency:" << mSettings.accelerationFreq <<
	"\nPressure Frequency:" << mSettings.pressureFreq <<
	"\nMagnetic Frequency:" << mSettings.magneticFreq <<
	"\nAltitude Frequency:" << mSettings.altitudeFreq <<
	"\nGyroscope Frequency:" << mSettings.gyroscopeFreq <<
	"\nVelocity Frequency:" << mSettings.velocityFreq 
	);
}

/**
 *  @brief Pulls the active configuration from the device.
 *  @details Fills mSettings from XsOutputConfigurationArray, the current
 *  active settings in the Xsens. 
 */
void mtiG::readSettings(){

	//Gets outputConfiguration from the device
	XsOutputConfigurationArray deviceConfig = device->outputConfiguration();

	//default initialization
	mSettings.orientationData = false; 
	mSettings.gpsData = false;
	mSettings.temperatureData = false;
	mSettings.accelerationData = false;
	mSettings.pressureData = false;
	mSettings.magneticData = false;
	mSettings.altitudeData = false;
	mSettings.gyroscopeData = false;
	mSettings.velocityData = false;


	for(int i =0; i < deviceConfig.size() ; i++)
	{
 			//ROS_DEBUG("%.8x", deviceConfig[i].m_dataIdentifier);
			switch(deviceConfig[i].m_dataIdentifier){
				// ORIENTATION
				case (XDI_Quaternion):
					mSettings.orientationData = true;
					break;
				// GPS
				case (XDI_LatLon):
				case (XDI_GpsAge):
				case (XDI_GpsDop):
				case (XDI_GpsSol):
					ROS_DEBUG("GPS ENABLED IN CURRENT CONFIGURATION");					
					mSettings.gpsData=true;			
					break;
				// TEMPERATURE
				case (XDI_Temperature):
					mSettings.temperatureData=true;			
					break;
				// ACCELERATION
				case (XDI_Acceleration):
					mSettings.accelerationData=true;			
					break;
				// PRESSURE
				case (XDI_BaroPressure):
					mSettings.pressureData=true;			
					break;
				// MAGNETIC
				case (XDI_MagneticField):
					mSettings.magneticData=true;			
					break;
				// ALTITUDE
				case (XDI_AltitudeEllipsoid):
					mSettings.altitudeData=true;			
					break;
				// GYROSCOPE 
				case (XDI_RateOfTurn):
					mSettings.gyroscopeData=true;			
					break;
				// VELOCITY
				case (XDI_VelocityXYZ):
					mSettings.velocityData=true;			
					break;

			}
	}

}

/**
 * @brief Creates an XsOutputConfigurationArray and pushes it to the sensor.
 * @details
 * - Configures the sensor with desired modules
 * - Refer to xsdataidentifier.h
 */
void mtiG::configure(){

	XsOutputConfigurationArray configArray;

	if(mSettings.orientationData){			//Quaternion - containsOrientation
			XsOutputConfiguration quat(XDI_Quaternion, mSettings.orientationFreq);// para pedir quaternion
			configArray.push_back(quat);
	}
	
	if(mSettings.gpsData){
			//LATITUDE E LONGITUDE -containsLatitudeLongitude
			XsOutputConfiguration gps(XDI_LatLon, mSettings.gpsFreq);// para pedir gps, //XDI_Quaternion 06/04
			configArray.push_back(gps);

			XsOutputConfiguration gps_age(XDI_GpsAge, mSettings.gpsFreq);// para pedir gps, //XDI_Quaternion 06/04
			configArray.push_back(gps_age);

			XsOutputConfiguration gps_sol(XDI_GpsSol, mSettings.gpsFreq);// para pedir gps, //XDI_Quaternion 06/04
			configArray.push_back(gps_sol);

			XsOutputConfiguration gps_dop(XDI_GpsDop, mSettings.gpsFreq);// para pedir gps, //XDI_Quaternion 06/04
			configArray.push_back(gps_dop);
	}
	
	if(mSettings.temperatureData){	
			//TEMPERATURA - containsTemperature
			XsOutputConfiguration temp(XDI_Temperature, mSettings.temperatureFreq);
			configArray.push_back(temp);
	}	
	
	if(mSettings.accelerationData){
			//ACCELERATION - containsCalibratedAcceleration
			XsOutputConfiguration accel(XDI_Acceleration, mSettings.accelerationFreq);
			configArray.push_back(accel);
	}

	if(mSettings.pressureData){	
			//PRESSURE - containsPressure
			XsOutputConfiguration baro(XDI_BaroPressure, mSettings.pressureFreq);
			configArray.push_back(baro);
	}

	if(mSettings.magneticData){
			//MAGNETIC FIELD - containsCalibratedMagneticField
			XsOutputConfiguration magnet(XDI_MagneticField, mSettings.magneticFreq);
			configArray.push_back(magnet);
	}

	if(mSettings.altitudeData){
			//ALTITUDE - containsAltitude
			XsOutputConfiguration alt(XDI_AltitudeEllipsoid, mSettings.altitudeFreq);
			configArray.push_back(alt);
	}

	if(mSettings.gyroscopeData){
			//GYRO - containsCalibratedGyroscopeData
			XsOutputConfiguration gyro(XDI_RateOfTurn, mSettings.gyroscopeFreq);
			configArray.push_back(gyro);
	}	

	if(mSettings.velocityData){
			//VELOCIDADE XYZ
			XsOutputConfiguration vel_xyz(XDI_VelocityXYZ, mSettings.velocityFreq);
			configArray.push_back(vel_xyz);
	}
	
	// Puts configArray into the device, overwriting the current configuration
	if (!device->setOutputConfiguration(configArray))
	{
			throw std::runtime_error("Could not configure MTmk4 device. Aborting.");
	}
}

/**
 * @brief Uses mSettings to advertise corresponding ROS Messages.
 * @details One message is published for each of the modules that are activated
 * in mSettings
 */
void mtiG::advertise(){
	
	ros::NodeHandle nh;
	int queue_size;
	ros::param::param<int>("~queue_size", queue_size, 50);
	if(mSettings.orientationData || mSettings.velocityData || mSettings.accelerationData){
	  	imuPublisher = nh.advertise<sensor_msgs::Imu> ("xsens/imu",queue_size);
		rpyPublisher = nh.advertise<geometry_msgs::Vector3Stamped> ("xsens/rpy", queue_size);
	}
	if(mSettings.gpsData){
		gpsPublisher = nh.advertise<sensor_msgs::NavSatFix> ("xsens/gps_data", queue_size);
		gpsInfoPublisher = nh.advertise<mtig_driver::GpsInfo> ("xsens/gps_extra", queue_size);
	}
	if(mSettings.velocityData){
		velPublisher = nh.advertise<geometry_msgs::TwistWithCovariance> ("xsens/velocity", queue_size);
	}
	if(mSettings.temperatureData){	
		tempPublisher = nh.advertise<sensor_msgs::Temperature>("xsens/temperature", queue_size);
	}
	if(mSettings.magneticData){
		magFieldPub = nh.advertise<sensor_msgs::MagneticField>("xsens/magnetic", queue_size);
	}
	if(mSettings.pressureData){	
		pressurePublisher = nh.advertise<sensor_msgs::FluidPressure>("xsens/pressure", queue_size);
	}

}

/**
 * @brief Uses mSettings and the MessageMaker class to publish corresponding ROS Messages.
 * @param packet - XsDataPacket from the Xsens API
 */
void mtiG::publish(){
	if(mSettings.orientationData || mSettings.velocityData || mSettings.accelerationData){
		imuPublisher.publish( messageMaker->fillImuMessage() );
	}
	if(mSettings.orientationData){
		rpyPublisher.publish( messageMaker->fillRPYMessage() );
	}
	if(mSettings.gpsData){
		gpsPublisher.publish( messageMaker->fillNavSatFixMessage() );
		gpsInfoPublisher.publish ( messageMaker->fillGpsInfoMessage() );
	}
	if(mSettings.velocityData){
		velPublisher.publish( messageMaker->fillVelocityMessage() );
	}
	if(mSettings.temperatureData){	
		tempPublisher.publish( messageMaker->fillTemperatureMessage() );
	}
	if(mSettings.magneticData){
		magFieldPub.publish( messageMaker->fillMagneticFieldMessage() );
	}
	if(mSettings.pressureData){	
		pressurePublisher.publish( messageMaker->fillPressureMessage() );
	}
}

/**
 * @brief  Initiates data extraction from packet obtained from the device.
 * @param packet - XsDataPacket from the Xsens API
 */
void mtiG::fillData(XsDataPacket * packet){
    sensorData.fillData(packet);
}
