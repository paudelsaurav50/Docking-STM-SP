#ifndef _TOPICS_H_
#define _TOPICS_H_

#include "rodos.h"

struct sLidarData
{
  int d[4] = {0, 0, 0, 0}; // Distance, mm
  float v[4] = {0, 0, 0, 0}; // Velocity, cm/s
  float dt = 0; // Thread period, mm
};

struct sCurrentData
{
  float i[4]; // Current, milli Amp
  float dt = 0; // Thread period, mm
};

extern Topic<sLidarData> LidarDataTopic;
extern Topic<sCurrentData> CurrentDataTopic;

#endif // telecommand.h
