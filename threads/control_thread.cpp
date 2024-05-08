#include "pid.h"
#include "rodos.h"
#include "topics.h"
#include "magnet.h"
#include "satellite_config.h"
#include "telecommand.h"

CommBuffer<sLidarData> LidarDataBuffer;
Subscriber LidarDataSubscriber(LidarDataTopic, LidarDataBuffer);
sLidarData LidarDataReceiver;

CommBuffer<sCurrentData> CurrentDataBuffer;
Subscriber CurrentDataSubscriber(CurrentDataTopic, CurrentDataBuffer);
sCurrentData CurrentDataReceiver;

float desired_current[4] = {0.0};
pid pid_distance;
float dist_sp = 25; // setpoint, mm

namespace RODOS {
extern HAL_UART uart_stdout;
}
#define TeleUART uart_stdout

class control_thread : public Thread
{
private:
  int period = PERIOD_CONTROL_MILLIS; // millis

public:
  control_thread(const char *thread_name) : Thread(thread_name) {}

  void init()
  {
    magnet::init();

    pid_distance.set_kp(PID_DISTANCE_KP);
    pid_distance.set_ki(PID_DISTANCE_KI);
    pid_distance.set_control_limits(PID_DISTANCE_UMIN, PID_DISTANCE_UMAX);
  }

  void run()
  {
    while (1)
    {
      // Read relative distance
      LidarDataBuffer.getOnlyIfNewData(LidarDataReceiver);
      CurrentDataBuffer.getOnlyIfNewData(CurrentDataReceiver);

      int d[4] = {LidarDataReceiver.lidar1, LidarDataReceiver.lidar2, LidarDataReceiver.lidar3, LidarDataReceiver.lidar4};
      int v[4] = {LidarDataReceiver.vel1, LidarDataReceiver.vel2, LidarDataReceiver.vel3, LidarDataReceiver.vel4};
      float i[4] = {CurrentDataReceiver.i0, CurrentDataReceiver.i1, CurrentDataReceiver.i2, CurrentDataReceiver.i3};

      float mean_dist = (d[0] + d[1] + d[2] + d[3]) / 4.0;

      // Perform position control
      float error = dist_sp - mean_dist;
      float current = pid_distance.update(error, period / 1000.0);

      desired_current[0] = current * 0;
      desired_current[1] = current * 0;
      desired_current[2] = current * 0;
      desired_current[3] = current * 0;

      // int a = 10;
      // float b = 10.0;
      PRINTF("$DAT= %f,%f,%f,%f,%f,%d,%d,%d,%d #)# \n", 22.0, i[0], i[1], i[2], i[3], LidarDataReceiver.lidar1, LidarDataReceiver.lidar2, LidarDataReceiver.lidar3, LidarDataReceiver.lidar4);
      // PRINTF("$DAT= %f,%f,%f,%f,%f,%d,%d,%d,%d #)# \r\n", b, b, b, b, b, a, a, a, a);
      // PRINTF("%f, %f, %f, %f\n", v[0], v[1], v[2], v[3]);
      // PRINTF("%d, %d, %d, %d, %f, %f\n", d[0], d[1], d[2], d[3], mean_dist, current);

      suspendCallerUntil(NOW() + period * MILLISECONDS);
    }
  }
};

char ReceiveData[MaxLength];
uint8_t Command(uint8_t TelecommandID)
{
	/*
	 * Execute the telecommands with respect to the telecommand ID
	 */
	char string[40];
	PRINTF("RECEIVED\n");
	// UART_Pi.write("RECEIVED\n",9);

	float a;
	int HBridgeENStatus;
	int Unit2ENStatus;
	int BatCHGENStatus;
	int DeploymentTime;
	float HBridgeValue;
	switch (TelecommandID){

	case TelemetryID:
		a = (float)(atof(ReceiveData));
		PRINTF("RECEIVED %f \n",a);
		return 1;
	case HBridgeUnit1EN:
    PRINTF("hehehehehhhhhhhhhhheeeeeeeeeeeee");
    if(int(atof(ReceiveData))==-1)
    {
      magnet::stop(MAGNET_IDX_ALL);
    }
    else
    {
      magnet::actuate(MAGNET_IDX_ALL, 50);
    }
		return 1;
	case Unit2EN:
		//Enable/Disable the second unit
		// Unit2ENStatus=int(atof(ReceiveData));
		// PRINTF("RECEIVED %d \n",Unit2ENStatus);
		// if(Unit2ENStatus==-1)
		// {
		// 	PRINTF("DISABLING EPS2 Unit\n ");
		// 	setENStatus(EPS2_EN,0);
		// }
		// else
		// 	setENStatus(EPS2_EN,1);
		return 1;
	case BATCHGEN:
		//Enable for Disable the Battery charging
		// BatCHGENStatus=int(atof(ReceiveData));
		// PRINTF("RECEIVED %d \n",BatCHGENStatus);
		// if(BatCHGENStatus==1)
		// {
		// 	PRINTF("Enabling Battery Charging\n ");
		// 	setENStatus(CHG_EN,0);
		// }
		// else
		// 	setENStatus(CHG_EN,1);
		return 1;
	case DeployCMD:
		//Command to deploy the antenna for given time in seconds
		DeploymentTime=int(atof(ReceiveData));
		PRINTF("RECEIVED %d \n",DeploymentTime);
		// thermalKnifeOON(Deployment_IN,DeploymentTime);
		return 1;
	case HBridge1PWM:
		//Sets the PWM for HBridge 1 (EM1)
		HBridgeValue=float(atof(ReceiveData));
		// EM_VAL_PWM1=HBridgeValue;
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case HBridge2PWM:
		//Sets the PWM for HBridge 2 (EM2)
		HBridgeValue=float(atof(ReceiveData));
		// EM_VAL_PWM2=HBridgeValue;
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case HBridge3PWM:
		//Sets the PWM for HBridge 3 (EM3)
		HBridgeValue=float(atof(ReceiveData));
		// EM_VAL_PWM3=HBridgeValue;
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case HBridge4PWM:
		//Sets the PWM for HBridge 3 (EM3)
		HBridgeValue=float(atof(ReceiveData));
		// EM_VAL_PWM4=HBridgeValue;
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case TakeImage:
		//Sends command to raspberry pi to take an image, just for the testing
			// PRINTF("RECEIVED to Take Image\n");
			// UART_Pi.write("$$TAM,CAM\n",10); ///Send this command to pi to take an image
			return 1;
	default:
		return 0;
	}
}


char TelecommandID;
uint8_t ReceiveState = 0;
uint8_t SignFlag = 0;
uint8_t	DotFlag = 0;
uint8_t DataIndex = 0;

uint8_t Decode(uint8_t RxBuffer)
{
	/*
		*Decode the Telecommands
		*/
	uint8_t success=0;
	// PRINTF("Test");
	switch (ReceiveState){

	case 0:
		SignFlag=0;
		DotFlag=0;
		DataIndex=0;
		if (RxBuffer==TelecommandStart)
		{
			//PRINTF("Test TC");
			ReceiveState=1;
		}
		break;

	case 1:
		SignFlag=0;
		DotFlag=0;
		DataIndex=0;
		if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		else {
			TelecommandID = RxBuffer;
			ReceiveState = 2;
		}
		break;

	case 2:
		if (RxBuffer=='+' || RxBuffer=='-')
		{
			if (SignFlag==0 && DataIndex==0)
				{
				SignFlag=1;
				ReceiveData[DataIndex]=RxBuffer;
				DataIndex++;
				ReceiveState = 2;
				}
			else {ReceiveState = 0;}
		}
		else if (RxBuffer=='.')
		{
			if (DotFlag==0)
				{
				DotFlag=1;
				ReceiveData[DataIndex]=RxBuffer;
				DataIndex++;
				ReceiveState = 2;
				}
			else {ReceiveState = 0;}
		}
		else if (RxBuffer>='0' && RxBuffer<='9')
		{
		    ReceiveData[DataIndex]=RxBuffer;
		    DataIndex++;
			if (DataIndex > MaxLength) {ReceiveState = 0;}
			else {ReceiveState = 2;}
		}
		else if (RxBuffer==TelecommandStart)
		{
			ReceiveState=1;
		}
		else if (RxBuffer==TelecommandStop)
		{
			ReceiveData[DataIndex]= 0x00;
			success=Command(TelecommandID);
			ReceiveState=0;
		}
		else { ReceiveState=0;}
		break;
	default:
		ReceiveState=0;
		break;
	}
	return success;
}


class Telecommand: public Thread {
public:

	void init() {



		//TelecommandLED.init(true, 1, 0);
	}
	void run(){
		char RxBuffer;
		while (1)
		{
			TeleUART.suspendUntilDataReady();
            //TelecommandLED.setPins(~TelecommandLED.readPins());

            TeleUART.read(&RxBuffer,1);
            Decode(RxBuffer);
		}
	}

}DockingEPS_TC_Thread;


control_thread tamariw_control_thread("control_thread");
