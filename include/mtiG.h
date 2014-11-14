/**
 * @class mtiG
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

#include <xsensdeviceapi.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <stdlib.h>
#include <getopt.h>
#include <string>


#include "sensorData.h"
#include "messageMaker.h"



#define THROTTLE_VALUE 10
#define FRAME_ID_STRING "xsens"




// Frame Id used in message headers

class mtiG{     
	private:	 
	int override;
	


	XsDevice * device;
	MessageMaker * messageMaker;
	SensorData sensorData;
	outputSettings mSettings;

	

	//Publishers
	ros::Publisher imuPublisher;
	ros::Publisher gpsPublisher;
	ros::Publisher velPublisher; 
	ros::Publisher tempPublisher;
	ros::Publisher magFieldPub;
	ros::Publisher pressurePublisher;
	ros::Publisher gpsInfoPublisher;
	ros::Publisher rpyPublisher;
		

	void printSettings();


	void parseOptions(int argc, char** argv);

	public:
		
	
	//Function
		
	//Constructors
	mtiG(XsDevice * _device );
	mtiG(XsDevice * _device, int argc, char ** argv);
	void configure();
	void fillData(XsDataPacket *);
		
	void advertise();
	void publish();
	void readSettings();
};
	
