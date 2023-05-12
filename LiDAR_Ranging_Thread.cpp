/*
 * Ranging.cpp
 *
 *  Created on: Nov 16, 2022
 *      Author: patha
 */

#include "rodos.h"
#include "VL53L4CD/VL53L4CD_api.h"
#include "hal.h"
#include "VL53L4CD/VL53L4CD_calibration.h"
#include "topics.h"
#include "VL53L4CD/MedianFilter.h"
#include "VL53L4CD/platform_TAMARIW.h"

HAL_I2C TOF_I2C(I2C_IDX1,GPIO_022,GPIO_023); //TAMARIW, I2C1: SCL:PB6, SDA:PB7

static Application module01("V_LIDAR", 2001);
#define NUM_ELEMENTS    9
sMedianFilter_t medianFilter1, medianFilter2, medianFilter3, medianFilter4;
sMedianNode_t medianBuffer1[NUM_ELEMENTS], medianBuffer2[NUM_ELEMENTS], medianBuffer3[NUM_ELEMENTS], medianBuffer4[NUM_ELEMENTS];

sLidarData LidarData = {
						0, 0, 0, 0,  //Lidar1 Lidar2 Lidar3 Lidar4
						0
						};
/*HAL_GPIO XSHUT1(GPIO_020);		//PB04
HAL_GPIO XSHUT2(GPIO_022);		//PB05
HAL_GPIO XSHUT3(GPIO_021);		//PB06
HAL_GPIO XSHUT4(GPIO_023);		//PB07
*/
class LIDAR : public Thread {
public:
		Dev_t 						dev;
		uint8_t 					status;
		uint8_t						loop;
		uint8_t						isReady;
		uint16_t					sensor_id;
		int16_t 					offset_mm;
		VL53L4CD_ResultsData_t 		results1,results2,results3,results4;
#define ADDR_ToF 0x29
#define MUX_ADDR 0x70
public:
	LIDAR (const char* name) : Thread(name){} // @suppress("Class members should be properly initialized")

	void init()
	{
		/*XSHUT1.init( true, 1, 0 );
		XSHUT2.init( true, 1, 0 );
		XSHUT3.init( true, 1, 0 );
		XSHUT4.init( true, 1, 0 );
		*/
		init4cd();
		dev=ADDR_ToF;
	}


	void setnewaddress(uint8_t addr) //changes the default address of the sensor to the desired address
	{
		status	= VL53L4CD_SetI2CAddress( dev, addr );
	}
	void calibrate()
	{
		/*XSHUT1.setPins(0);
		XSHUT2.setPins(0);
		XSHUT3.setPins(0);
		XSHUT4.setPins(0);

		suspendCallerUntil(NOW()+10*MILLISECONDS);
		XSHUT1.setPins(1);
		suspendCallerUntil(NOW()+10*MILLISECONDS);
		setADDR(ADDR1);
		*/
		status=VL53L4CD_GetSensorId(dev, &sensor_id);
		if(status || (sensor_id=0xEBAA))
		{
			PRINTF("VL53L4CD detected, Status=%6u, Sensor_ID= %x, Device Address= %x \n", status, sensor_id, dev);
		}
		if(status)
				{
					PRINTF("VL53L4Cd ULD loading failed \n");
				}
					PRINTF ("Sensor Ready \n");
					status = VL53L4CD_SensorInit( dev );
					PRINTF ("Sensor Initialized \n");
					status = VL53L4CD_StartRanging(dev);
		status = VL53L4CD_CalibrateOffset(dev, 100, &offset_mm, 20);
			if(status)
			{
				PRINTF("VL53L4CD_CalibrateOffset failed with status %u\n", status);
			}

			PRINTF("Offset calibration done, offset value = %d mm\n", offset_mm);
	}

	void rangestart() //used to start the Lidar once called by run1sensor
	{
		medianFilter1.numNodes = NUM_ELEMENTS;
		medianFilter1.medianBuffer = medianBuffer1;
		MEDIANFILTER_Init(&medianFilter1);
		status=VL53L4CD_GetSensorId(dev, &sensor_id);
		if(status || (sensor_id=0xEBAA))
		{
			PRINTF("VL53L4CD detected, Status=%6u, Sensor_ID= %x, Device Address= %x \n", status, sensor_id, dev);
		}
		if(status)	PRINTF("VL53L4Cd ULD loading failed \n");
		else{
					PRINTF ("Lidar Sensor Ready \n");
					status = VL53L4CD_SensorInit( dev );
					PRINTF ("Lidar Sensor Initialized \n");
					status = VL53L4CD_StartRanging(dev);
					loop= 0;
					while (loop<300)
					{
					status = VL53L4CD_CheckForDataReady(dev, &isReady);
					if (isReady)
						VL53L4CD_ClearInterrupt(dev);
						VL53L4CD_GetResult(dev, &results1);
						//PRINTF("Device_ID= %x, status1= %6u,Distance1=%6u,Signal1=%6u \n", dev, results1.range_status, results1.distance_mm, results1.signal_per_spad_kcps);
						LidarData.lidar1 = MEDIANFILTER_Insert(&medianFilter1, results1.distance_mm);
						//PRINTF("Lidar_Distance: %d \t Filtered_Value: %d\r\n", results1.distance_mm, LidarData.lidar1);
						if(LidarData.lidar1==0)
							LidarData.lidar1=results1.distance_mm; //until median filter is not in action
						//PRINTF("Distance_LiDAR: %d \n", LidarData.lidar1);
						LidarDataTopic.publish(LidarData);
						loop++;
						suspendCallerUntil(NOW()+ 200*MILLISECONDS);

					}
		}
					//PRINTF("Ranging Done. GO HOME NOW!!!!\n");
	}
	void runToFSensor(uint8_t portNo)
	{
		status=PCA9546_SelPort(portNo,(uint16_t)MUX_ADDR);
		//setADDR(ADDR1);
		if(status)	PRINTF("Failed setting port \n");
		status=VL53L4CD_SetRangeTiming(dev , 200 , 0);
		if (status)	PRINTF("VL53L4CD_Set range timing failed \n");
		status=VL53L4CD_SetRangeTiming(dev , 200 , 0);
		if (status)	PRINTF("VL53L4CD_Set range timing failed\n");
		rangestart();
	}
	void run()
		{
			//run4sensors();
			//run3sensors();
			//run2sensors();
			//calibrate();
			runToFSensor(1);
		}
};
LIDAR LIDAR ("LIDAR");

