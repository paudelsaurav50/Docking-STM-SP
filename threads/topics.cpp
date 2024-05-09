/*****************************************************************
topics.cpp

Original Created by: Atheel Redah @ University of W�rzburg
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + W�rzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
*****************************************************************/

#include "topics.h"

Topic<sTelecommandData> TelecommandDataTopic(-1, "Telecommand Data");
Topic<sLidarData> LidarDataTopic(-1, "Lidar Data");
Topic<sCurrentData> CurrentDataTopic(-1, "Current Data");
