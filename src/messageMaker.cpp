/**
 * @file messageMaker.cpp
 * @brief Fills ROS messages with data from the sensor
 * @details This class will be constantly used by mtiG in order to publish
 *          ROS messages containing information about the Xsens device sensors
 *          data.
 * @author Lucas Casanova Nogueira
 * @date 2014
 */

#include "messageMaker.h"

/**
 * @brief Initializes holder for SensorData object
 * @param _data - a SensorData object
 */
MessageMaker::MessageMaker(SensorData & _data):data(_data){
}

/**
 * @brief Creates a sensor_msgs/Imu ROS message from sensor data    	
 * @details Uses data collected from the device's inertial measurement unit(IMU)
 * angular velocity from gyroscope
 * linear acceleration from accelerometer
 * orientation fusioned from the magnetometer
 * error is set using values from the <a href="https://www.xsens.com/wp-content/uploads/2013/12/MTi-User-Manual.pdf">MTi User Manual</a>
 */
sensor_msgs::Imu MessageMaker::fillImuMessage(){
	sensor_msgs::Imu imuData;
	imuData.header.frame_id = data.frameId();
	imuData.header.stamp = ros::Time::now();
	
	imuData.angular_velocity.x = data.gyroscope_x();
	imuData.angular_velocity.y = data.gyroscope_y();
	imuData.angular_velocity.z = data.gyroscope_z();

	for(int i=0 ; i < 9 ; i++){
		imuData.angular_velocity_covariance[i]=0.0;
	}
	imuData.angular_velocity_covariance[0] = 
		imuData.angular_velocity_covariance[4] =
		imuData.angular_velocity_covariance[8] = data.gyrError();

	imuData.linear_acceleration.x = data.accelerometer_x();
	imuData.linear_acceleration.y = data.accelerometer_y();
	imuData.linear_acceleration.z = data.accelerometer_z();

	for(int i=0 ; i < 9 ; i++){
		imuData.linear_acceleration_covariance[i]=0.0;
	}
	imuData.linear_acceleration_covariance[0] = 
		imuData.linear_acceleration_covariance[4] = 
		imuData.linear_acceleration_covariance[8] = data.accError();


	imuData.orientation.x = data.quaternion_x();
	imuData.orientation.y = data.quaternion_y();
	imuData.orientation.z = data.quaternion_z();
	imuData.orientation.w = data.quaternion_w();

	for(int i=0 ; i < 9 ; i++){
		imuData.orientation_covariance[i]=0.0;
	}
	imuData.orientation_covariance[0] = data.rollError();
	imuData.orientation_covariance[4] = data.pitchError();
	imuData.orientation_covariance[8] = data.yawError();
 	
	return imuData;
}

/**
 * @brief Creates a sensor_msgs/NavSatFix ROS message from sensor data
 * @details Uses data collected from the device's GPS. 
 *           The covariance is given using DOP and Position Acuraccy.
 *           We assumed Position Accuracy is the std dev.
 */
sensor_msgs::NavSatFix MessageMaker::fillNavSatFixMessage(){
	sensor_msgs::NavSatFix gpsData;
	sensor_msgs::NavSatStatus status_msg;
	
	gpsData.header.frame_id = data.frameId() + "_gps" ;
	gpsData.header.stamp = ros::Time::now();

	gpsData.altitude = data.altitude();
	gpsData.latitude = data.latitude();
	gpsData.longitude = data.longitude();

	gpsData.position_covariance_type = sensor_msgs::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
	
	for(int i=0 ; i < 9 ; i++){
		gpsData.position_covariance[i]=0.0;
	}

	//Covariance obtained using DOP and PositionAccuracy from xsrawgpssol.h in the xsens api
	gpsData.position_covariance[0]= (data.edop() * data.PositionAccuracy()/100) * (data.edop() * data.PositionAccuracy()/100);
	gpsData.position_covariance[4]= (data.ndop() * data.PositionAccuracy()/100) * (data.ndop() * data.PositionAccuracy()/100);
	gpsData.position_covariance[8]= (data.vdop() * data.PositionAccuracy()/100) * (data.vdop() * data.PositionAccuracy()/100);

	//Status comes from status word of the sensor, no augmentation at this step
	if(data.GpsFixStatus()==4){
		status_msg.status = sensor_msgs::NavSatStatus::STATUS_FIX;
	}
	else{
		status_msg.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
	}

	gpsData.status = status_msg;

	return gpsData;
}


/**
 * @brief Creates a geometry_msgs/Twist ROS message from sensor data
 * @details Publishes angular and linear speed information in the ENU frame of reference.
 * The data is collected from the device's gyroscopes and GPS
 */
geometry_msgs::TwistWithCovariance MessageMaker::fillVelocityMessage(){
	geometry_msgs::TwistWithCovariance res;	
	geometry_msgs::Twist velocityData;
	for(int i=0;i<36;i++){
		res.covariance[i]=0.0;
	} 
	res.covariance[0]=res.covariance[7]=res.covariance[14]=0.001;
	res.covariance[21]=res.covariance[28]=res.covariance[35]=0.01;
	//conversion of angular velocity to ENU frame of reference
	tf::Quaternion oq;
	data.getOrientationQuaternion(&oq);
	
	tf::Quaternion local_w(data.gyroscope_x(), data.gyroscope_y(), data.gyroscope_z(), 1.0);
	tf::Quaternion global_w = oq * local_w * oq.inverse();
	//RPY in ENU frame
	double groll, gpitch, gyaw;
	tf::Matrix3x3(global_w).getRPY(groll,gpitch, gyaw);
	velocityData.angular.x = groll;
	velocityData.angular.y = gpitch;
	velocityData.angular.z = gyaw;

	velocityData.linear.x = data.velocity_x();
	velocityData.linear.y = data.velocity_y();
	velocityData.linear.z = data.velocity_z();
	res.twist=velocityData;

	return res;


}

/**
 * @brief Creates a sensor_msgs/Temperature ROS message from sensor data
 * @details Uses data collected from the device's thermometer.
 */
sensor_msgs::Temperature MessageMaker::fillTemperatureMessage(){
	sensor_msgs::Temperature tempData;
	
	tempData.header.frame_id = data.frameId();
	tempData.header.stamp = ros::Time::now();

	tempData.temperature = data.temperature();

	return tempData;

}

/**
 * @brief Creates a sensor_msgs/FluidPressure ROS message from sensor data
 * @details Uses data collected from the device's barometer.
 */
sensor_msgs::FluidPressure MessageMaker::fillPressureMessage(){
	sensor_msgs::FluidPressure pressureData;
	
	pressureData.header.frame_id = data.frameId();
	pressureData.header.stamp = ros::Time::now();

	pressureData.fluid_pressure =  data.pressure();

	return pressureData;

}

/**
 * @brief Creates a sensor_msgs/MagneticField ROS message from sensor data    
 * @details Uses data collected from the device's magnetometer
 * @see SensorData::magnetic_x()	
 */
sensor_msgs::MagneticField MessageMaker::fillMagneticFieldMessage(){
	sensor_msgs::MagneticField magneticData;
	
	magneticData.header.frame_id = data.frameId();
	magneticData.header.stamp = ros::Time::now();

	magneticData.magnetic_field.x = data.magnetic_x();
	magneticData.magnetic_field.y = data.magnetic_y();
	magneticData.magnetic_field.z = data.magnetic_z();

	return magneticData;
}

geometry_msgs::Vector3Stamped MessageMaker::fillRPYMessage(){
	geometry_msgs::Vector3Stamped rpyData;
	rpyData.header.frame_id = data.frameId();
	rpyData.header.stamp = ros::Time::now();
	
	rpyData.vector.x = data.roll();
	rpyData.vector.y = data.pitch();
	rpyData.vector.z = data.yaw();
	
	return rpyData;
}

/**		
 * @brief Creates a custom mtig_driver/GpsInfo ROS message from sensor data  
 * @details This message contains important information from the Xsens' GPS
 * that no other ROS Message accounted for. 
 */
mtig_driver::GpsInfo MessageMaker::fillGpsInfoMessage(){
	mtig_driver::GpsInfo gps_info;
	gps_info.header.frame_id = data.frameId();
	gps_info.header.stamp = ros::Time::now();


	//DOP INFORMATION
	gps_info.geometricDOP = data.gdop();
	gps_info.positionDOP = data.pdop();
	gps_info.timeDOP = data.tdop();
	gps_info.verticalDOP = data.vdop();
	gps_info.horizontalDOP = data.hdop();
	gps_info.eastingDOP = data.edop();
	gps_info.northingDOP = data.ndop();
	 
	gps_info.itow = data.itow();

	//POSITION AND SPEED ACCURACY
	gps_info.position_accuracy = data.PositionAccuracy();
	gps_info.speed_accuracy = data.SpeedAccuracy();
	
	//NUMBER OF SATELLITES USED IN GPS ACQUISITION
	gps_info.satellite_number = data.SatelliteNumber();

	gps_info.gps_fix = data.GpsFixStatus();


	return gps_info;
}
