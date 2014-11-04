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

#include "xsensdeviceapi.h"

/*! \brief Moves the console cursor
	\param x : The new x position (column) of the cursor
	\param y : The new y position (row) of the cursor
*/
void gotoXY(int x, int y)
{
#ifdef WIN32
	COORD coord;
	coord.X = x;
	coord.Y = y;
	SetConsoleCursorPosition(GetStdHandle(STD_OUTPUT_HANDLE), coord);
#else
	char essq[100];		// String variable to hold the escape sequence
	char xstr[100];		// Strings to hold the x and y coordinates
	char ystr[100];		// Escape sequences must be built with characters

	/*
	** Convert the screen coordinates to strings
	*/
	sprintf(xstr, "%d", x);
	sprintf(ystr, "%d", y);

	/*
	** Build the escape sequence (vertical move)
	*/
	essq[0] = '\0';
	strcat(essq, "\033[");
	strcat(essq, ystr);

	/*
	** Described in man terminfo as vpa=\E[%p1%dd
	** Vertical position absolute
	*/
	strcat(essq, "d");

	/*
	** Horizontal move
	** Horizontal position absolute
	*/
	strcat(essq, "\033[");
	strcat(essq, xstr);
	// Described in man terminfo as hpa=\E[%p1%dG
	strcat(essq, "G");

	/*
	** Execute the escape sequence
	** This will move the cursor to x, y
	*/
	printf("%s", essq);
#endif
}

/*! \brief clears the console screen
*/
void clearScreen()
{
#ifdef WIN32
	CONSOLE_SCREEN_BUFFER_INFO csbi;
	HANDLE hStdOut = GetStdHandle(STD_OUTPUT_HANDLE);
	COORD coord = {0, 0};
	DWORD count;

	GetConsoleScreenBufferInfo(hStdOut, &csbi);
	FillConsoleOutputCharacter(hStdOut, ' ', csbi.dwSize.X * csbi.dwSize.Y, coord, &count);
	SetConsoleCursorPosition(hStdOut, coord);
#else
	int i;

	for (i = 0; i < 100; i++)
		// Insert new lines to create a blank screen
		putchar('\n');
	gotoXY(0,0);
#endif
}

