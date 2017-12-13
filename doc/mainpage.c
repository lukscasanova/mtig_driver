/**
 * \mainpage MTi-G-700 GPS ROS driver
 * 
 * \author Lucas Casanova Nogueira, based on code from Ji Zhang and Silvio Maeta.
 * \date 2014
 * \copyright BSD
 * \section Overview
 * 
 * This is a ROS driver for MTi-G-700 GPS/INS. It provides GPS fix reading 
 * capabilities, as well as IMU and other sensors. To use the driver, a license
 * number is required for the MT Software Suite (a.k.a. the Xsens API). This 
 * driver also enables configuration, i.e., turn different sensors in the device
 * on and off, via ROS node. It is also possible to adjust the settings using
 * MT Manager in Windows, and using the driver in read-only mode.
 * 
 * \section Classes
 * 
 * This code is based mainly in three classes, which are:
 * - mtiG
 * - SensorData
 * - MessageMaker
 * 
 * The mtiG class represents a device. It maintains an XsDevice object from which
 * it gets the data packets (XsDataPackets). It uses a SensorData object to hold 
 * all the data available in each data packet. Later, it calls MessageMaker
 * functions to fill ROS Messages from the data stored in SensorData, and then 
 * mtiG publishes this messages.
 *
 */


