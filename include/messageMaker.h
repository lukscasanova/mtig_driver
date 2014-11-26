/**
 * @class MessageMaker
 * @brief Fills ROS messages with data from the sensor
 * @details This class will be constantly used by mtiG in order to publish
 *          ROS messages containing information about the Xsens device sensors
 *          data.
 * @author Lucas Casanova Nogueira (lucas_afonso@hotmail.com)
 * @date 2014
 */

#ifndef MESSAGE_MAKER
#define MESSAGE_MAKER

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf/tf.h>
#include "mtig_driver/GpsInfo.h"

#include "sensorData.h"

class MessageMaker{

private:
  SensorData & data;
  
public:
  MessageMaker(SensorData & data);
  sensor_msgs::Imu fillImuMessage();
  sensor_msgs::NavSatFix fillNavSatFixMessage();
  geometry_msgs::TwistWithCovariance fillVelocityMessage();
  sensor_msgs::Temperature fillTemperatureMessage();
  sensor_msgs::FluidPressure fillPressureMessage();
  sensor_msgs::MagneticField fillMagneticFieldMessage();
  mtig_driver::GpsInfo fillGpsInfoMessage();
  geometry_msgs::Vector3Stamped fillRPYMessage();
   
};

#endif
