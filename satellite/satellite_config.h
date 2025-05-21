#ifndef _SATELLITE_CONFIG_H_
#define _SATELLITE_CONFIG_H_

#include "sat_config.h"

#ifdef GOLD_SAT
#include "config_gold.h"
#endif

#ifdef BLACK_SAT
#include "config_black.h"
#endif

#ifdef WHITE_SAT
#include "config_white.h"
#endif

#ifdef SILVER_SAT
#include "config_silver.h"
#endif

#endif // satellite_config.h
