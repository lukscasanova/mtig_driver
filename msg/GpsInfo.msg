Header header

#GPS DOP
float64 geometricDOP
float64 positionDOP
float64 timeDOP
float64 verticalDOP
float64 horizontalDOP
float64 northingDOP
float64 eastingDOP
float64 itow


#Position Accuracy Estimate
float64 position_accuracy
float64 speed_accuracy

#Number of satellites
uint8 satellite_number

uint8 GPS_NO_FIX = 0
uint8 DEAD_RECKONING = 1
uint8 GPS_2D_FIX = 2
uint8 GPS_3D_FIX = 3
uint8 GPS_AND_DEAD_RECKONING = 4

uint8 gps_fix
