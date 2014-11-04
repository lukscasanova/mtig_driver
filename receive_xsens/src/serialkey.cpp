/**
 * @file serialkey.cpp
 * @brief Used to validate the serial key of your device
 * @details This file is a part of the Xsens API and used to validate
 *          your device. You can either insert your serial key here or
 *          in serialkey.h
 */

/* 
   @copyright 
     Copyright (c) Xsens Technologies B.V., 2006-2012. All rights reserved.
 
      This source code is provided under the MT SDK Software License Agreement 
       and is intended for use only by Xsens Technologies BV and
       those that have explicit written permission to use it from
       Xsens Technologies BV.
 
      THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
       KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
       IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
       PARTICULAR PURPOSE.
 */

#include "serialkey.h"
#include <xsens/xsstring.h>
#include <xsens/xscontrol.h>
#include <stdio.h>
#include <iostream>

#define SERIAL_KEY_SIZE	30

extern "C"
int setSerialKey()
{
	if (strcmp(SERIAL_KEY, "enter serial key here") == 0)
	{
		{
			char serialKey[256];
			FILE* fp;
			memset(serialKey, 0, 256);
			fp = fopen("serial.key", "r");
			if (fp)
			{
				size_t result = fread(serialKey, 1, SERIAL_KEY_SIZE, fp);
				fclose(fp);

				if (result == SERIAL_KEY_SIZE)
				{           
                    if (XsControl::setSerialKey(serialKey))
                        return 1;
                }
			}
		}

		// ask for serial key
		std::cout << "Please enter valid serial key" << std::endl;
		std::cout << "If you built this example yourself you can enter the key in \"serialkey.h\"" << std::endl;
		char serialKey[256];
		std::cin.getline(serialKey, 256);
		XsString serial(serialKey);
		
		if (XsControl::setSerialKey(serial))
		{
			// store it
			FILE* fp = fopen("serial.key", "w");
			fwrite(serial.c_str(), sizeof(char), serial.size(), fp);
			fclose(fp);
			return 1;
		}
		return 0;
	}
	return XsControl::setSerialKey(SERIAL_KEY);
}

