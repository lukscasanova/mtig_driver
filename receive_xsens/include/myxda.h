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

#ifndef MYXDA_H
#define MYXDA_H

#include <xsens/xscallback.h>
#include <map>

struct XsControl;
struct XsDevice;

class MyXda : public XsCallback
{
public:
	MyXda();
	virtual ~MyXda();

	void openPorts();

	int getUserUpdateRate();
	void setUpdateRate(int updateRate);

	enum DisplayMode
	{
		DM_None,
		DM_OrientationEuler,
		DM_OrientationQuaternion,
		DM_Sdi,
		DM_OrientationMatrix,		
	};

	void setDisplayMode(DisplayMode displayMode);
	inline XsControl* xsControl() const { return m_xsControl; }

protected:
	virtual void onDataAvailable(XsDevice*, const XsDataPacket* packet);

private:
	void clearMeasurementScreen();
	void printDisplayMode();

	XsControl* m_xsControl;
	int m_connectCount;
	std::map<uint32_t, int> m_connectedDevices;
	volatile bool m_stationReady;
	DisplayMode m_displayMode;
	DisplayMode m_previousDisplayMode;
};

#endif	// MYXDA_H
