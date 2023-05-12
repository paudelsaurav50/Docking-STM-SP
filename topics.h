/*****************************************************************
topics.h

Original Created by: Atheel Redah @ University of Würzburg
Original Creation Date: March 8, 2015

Development environment specifics:
	Software Platform: Rodos (Realtime Onboard Dependable Operating System).
	Hardware Platform: STM32F4 + Würzburg Uni Informatik 8 Discovery AddOn Board Version 2.0.
*****************************************************************/

#ifndef __topics_h__
#define __topics_h_

/* Includes ------------------------------------------------------------------*/
#include "matlib.h"

/* Exported types ------------------------------------------------------------*/

struct sTelecommandData
{
	bool Telemetry;
	float DistanceUWB;
};

struct sLidarData{
	 int16_t lidar1, lidar2, lidar3, lidar4; //Filtered Distance from Lidar1, Lidar2, lidar3, Lidar4
     double deltaTime;
   };

struct sCalculation
{

	float Velocity;
	float Position;
	float acceleration;
};

struct Orientation{
	float yaw, pitch;
};

extern Topic<sTelecommandData>  TelecommandDataTopic;
extern Topic<sLidarData> 		LidarDataTopic;

#endif

