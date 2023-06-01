/*
 * DockingEPSTestings.cpp
 *
 * Created on: 17.02.2023
 * Ref From: Atheel Redah
 * Created By: Saurav Paudel
 */

#include "rodos.h"
#include <stdio.h>
#include <stdlib.h>
#include "hal.h"
#include "math.h"
#include "topics.h"
#include "telecommand.h"

CommBuffer<sTelecommandData> SensorsTelecommandDataBuffer;
Subscriber SensorsTelecommandDataSubscriber(TelecommandDataTopic, SensorsTelecommandDataBuffer);
sTelecommandData TelecommandDataReceiver;

CommBuffer<sLidarData> LidarDataBuffer;
Subscriber LidarDataSubscriber(LidarDataTopic, LidarDataBuffer);
sLidarData LidarDataReceiver;
//Telecommand Thread by Atheel
static Application module01("Atheel2", 2001);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t ReceiveState = 0;
uint8_t SignFlag = 0;
uint8_t	DotFlag = 0;
uint8_t DataIndex = 0;
char TelecommandID;
char ReceiveData[MaxLength];
/* Private function prototypes -----------------------------------------------*/
/* PTRU Components 1570694 Litze LiY 1 x 0.14mm² rotrivate functions ---------------------------------------------------------*/

//extern HAL_GPIO TelecommandLED;


namespace RODOS {
extern HAL_UART uart_stdout;
}
#define TeleUART uart_stdout

//Define primary unit
#define UNIT_PRIMARY true //true: Primary unit, false: Secondary unit

//All GPIOs required for EPS
#define EN_EPS_1 GPIO_058 //PD10, Enable EPS1
#define EN_EPS_2 GPIO_059 //PD11, Enable EPS2
#define EN_HBRIDGE_1 GPIO_056   //PD8, Enable HBridge 1
#define EN_HBRIDGE_2 GPIO_057   //PD9, Enable HBridge 2
#define EN_CHG_BAT GPIO_038 //PC6, Charging Enable Pin
#define DEP_IN GPIO_043 //PC11, Thermal Knife

#define IN4_2 GPIO_073 //PE9, HBridge 4 input 2
#define IN4_1 GPIO_074 //PE10, HBridge 4 input 1
#define IN3_2 GPIO_075 //PE11, HBridge 3 input 2
#define IN3_1 GPIO_076 //PE12, HBridge 3 input 1
#define IN2_2 GPIO_060 //PD12, HBridge 2 input 2
#define IN2_1 GPIO_061 //PD13, HBridge 2 input 1
#define IN1_2 GPIO_062 //PD14, HBridge 1 input 2
#define IN1_1 GPIO_063 //PD15, HBridge 1 input 1

#define PWM1 PWM_IDX04
#define PWM2 PWM_IDX05
#define PWM3 PWM_IDX06
#define PWM4 PWM_IDX07

#define OCC1_CH ADC_CH_010 //ADC3_CH10, PC0, HBridge1 current monitor
#define OCC2_CH ADC_CH_011 //ADC3_CH11, PC1, HBridge2 current monitor
#define OCC3_CH ADC_CH_012 //ADC3_CH12, PC2, HBridge3 current monitor
#define OCC4_CH ADC_CH_013 //ADC3_CH13, PC3, HBridge4 current monitor
#define ADC_NO ADC_IDX3 //Using ADC 3 for PC0,1,2 and 3

#define BATT_MES_ADC_CH ADC_CH_014 //ADC3_CH14, PC4, Battery Voltage Monitor
#define ADC_NO_BAT_MES ADC_IDX1 //Using ADC 1 for PC4

HAL_GPIO EPS1_EN(EN_EPS_1); //EPS 1 Enable HAL GPIO Defn
HAL_GPIO EPS2_EN(EN_EPS_2); //EPS 2 Enable HAL GPIO Defn
HAL_GPIO EPS1_HBridge_EN(EN_HBRIDGE_1); //EPS 1 HBridge HAL GPIO Defn
HAL_GPIO CHG_EN(EN_CHG_BAT); //Charge Enable HAL GPIO Defn
HAL_GPIO Deployment_IN(DEP_IN); //Thermal Knife pin HAL GPIO Defn

//Define HAL GPIO for LEDs
HAL_GPIO LED0(GPIO_048);
HAL_GPIO LED1(GPIO_049);
HAL_GPIO LED2(GPIO_050);
HAL_GPIO LED3(GPIO_051);
HAL_GPIO LED4(GPIO_052);
HAL_GPIO LED5(GPIO_053);
HAL_GPIO LED6(GPIO_054);
HAL_GPIO LED7(GPIO_055);

//Define HAL GPIO for HBrigdge input pins
HAL_GPIO  HBridge1_IN1(IN1_1);
HAL_GPIO  HBridge1_IN2(IN1_2);
HAL_GPIO  HBridge2_IN1(IN2_1);
HAL_GPIO  HBridge2_IN2(IN2_2);
HAL_GPIO  HBridge3_IN1(IN3_1);
HAL_GPIO  HBridge3_IN2(IN3_2);
HAL_GPIO  HBridge4_IN1(IN4_1);
HAL_GPIO  HBridge4_IN2(IN4_2);

//Define HAL PWM for EM PWM
HAL_PWM EM_PWM1(PWM1);
HAL_PWM EM_PWM2(PWM2);
HAL_PWM EM_PWM3(PWM3);
HAL_PWM EM_PWM4(PWM4);

//Degine HAL UART for the UART communication with Rasp pi
HAL_UART UART_Pi(UART_IDX3, GPIO_026, GPIO_027);

//Define HAL ADC for EM current measurement
HAL_ADC EM_ADC(ADC_NO);

//Define HAL ADC for battery measurement
HAL_ADC BATT_ADC(ADC_NO_BAT_MES);

void setPWM(HAL_PWM pin, float Value)
{
	/*
	 * @Input: HAL_PWM type and the value
	 * This function sets the PWM to the HBridge.
	 */
	 PRINTF("value=%f",Value);
	if(Value>=0 && Value<1024)
		pin.write((unsigned int) Value);
	else PRINTF("Between -1023 and 1023 please");

}

void setENStatus(HAL_GPIO pin,int Value)
{
	/*
		 * @Input: HAL_GPIO type and the value
		 * This function can be used to set the enable status for the pin.
	*/
	if(Value==1 || Value==0)
		 pin.setPins(Value);
	else PRINTF("Wrong Value");
}

void thermalKnifeOON(HAL_GPIO pin,int Time_s)
{
	/*
		 * @Input: HAL_GPIO type and time in seconds for the deployment.
		 * This function turns ON the thermal knife with 2 seconds interval.
	*/
		PRINTF("DEP INITIATED for %d ms \n",Time_s);
		double initTme=SECONDS_NOW();
		double fourSecondsTime=SECONDS_NOW();
		PRINTF("DEP TIME %d \n", (int)(SECONDS_NOW()-initTme));
		int diffTime=(int)(SECONDS_NOW()-initTme);

		/*
		 * For the deployment time given,
		 * Turn it ON for 2s and Turn it OFF for next 2 seconds
		 * Pulsed Mode
		 */
		while(diffTime<=Time_s)
		{

			if((SECONDS_NOW()-fourSecondsTime)<=2)
			{
				pin.setPins(1);
				PRINTF("HIGH \n");
			}
			else if((SECONDS_NOW()-fourSecondsTime)>2 && (SECONDS_NOW()-fourSecondsTime)<=4)
			{
				PRINTF("LOW \n");
				pin.setPins(0);
			}
			else
			{
				fourSecondsTime=SECONDS_NOW();
			}
			diffTime=(int)(SECONDS_NOW()-initTme);
		}
		pin.setPins(0);
}
void setHBridge(int HBrdigeNo, float Value)
{
	/*
		 * @Input: HBridge No(1,2,3,4) type and the value(-1023 to 1023))
	*/
	switch(HBrdigeNo)
		{
		case 1:
			//RUN First HBridge
			if(Value<0)
			{
				setPWM(EM_PWM1, -1*Value);
				HBridge1_IN1.setPins(1);
				HBridge1_IN2.setPins(0);
			}
			else
			{
				setPWM(EM_PWM1, Value);
				HBridge1_IN1.setPins(0);
				HBridge1_IN2.setPins(1);
			}
			break;
		case 2:
			//RUN 2nd HBridge
			if(Value<0)
			{
				setPWM(EM_PWM2, -1*Value);
				HBridge2_IN1.setPins(1);
				HBridge2_IN2.setPins(0);
			}
			else
			{
				setPWM(EM_PWM2, Value);
				HBridge2_IN1.setPins(0);
				HBridge2_IN2.setPins(1);
			}
			break;
		case 3:
			//RUN 3rd HBridge
			if(Value<0)
			{
				setPWM(EM_PWM3, -1*Value);
				HBridge3_IN1.setPins(1);
				HBridge3_IN2.setPins(0);
			}
			else
			{
				setPWM(EM_PWM3, Value);
				HBridge3_IN1.setPins(0);
				HBridge3_IN2.setPins(1);
			}
			break;
		case 4:
			//RUN 4th HBridge
			if(Value<0)
			{
				setPWM(EM_PWM4, -1*Value);
				HBridge4_IN1.setPins(1);
				HBridge4_IN2.setPins(0);
			}
			else
			{
				setPWM(EM_PWM4, Value);
				HBridge4_IN1.setPins(0);
				HBridge4_IN2.setPins(1);
			}
			break;

		}
}


void getEMCurrent()
{
	/*
	 *@Prints: Current through Electromagnets
		*/
	//Voltage from current measurement pin
	float adcValOCM1 = ((float(EM_ADC.read(OCC1_CH)))/4096)*3290;
	float adcValOCM2 = ((float(EM_ADC.read(OCC2_CH)))/4096)*3290;
	float adcValOCM3 = ((float(EM_ADC.read(OCC3_CH)))/4096)*3290;
	float adcValOCM4 = ((float(EM_ADC.read(OCC4_CH)))/4096)*3290;
	//Convert the voltage to the current from the relation given in the datasheet
	float currentEM1=(adcValOCM1/140);
	float currentEM2=(adcValOCM2/140);
	float currentEM3=(adcValOCM3/140);
	float currentEM4=(adcValOCM4/140);
	/*uint16_t adcValOCM1 = EM_ADC.read(OCC1_CH);
	uint16_t adcValOCM2 = EM_ADC.read(OCC2_CH);
	uint16_t adcValOCM3 = EM_ADC.read(OCC3_CH);
	uint16_t adcValOCM4 = EM_ADC.read(OCC4_CH);
	*/
	PRINTF("I1= %f A,I2= %f A,I3= %f A,I4= %f A \n",currentEM1,currentEM2,currentEM3,currentEM4);
	char uart_data[100];
	sprintf(uart_data,"I1= %f A,I2= %f A,I3= %f A,I4= %f A \n",currentEM1,currentEM2,currentEM3,currentEM4);
	UART_Pi.write(uart_data,strlen(uart_data));
}

void getBATVoltage()
{
	/*
		 *@Prints: Battery voltage
	*/
	float adcValBAT = ((float(BATT_ADC.read(BATT_MES_ADC_CH)))/4096)*3.3;
	PRINTF("adcValBAT= %f V\n",adcValBAT);
	float battVoltage=(adcValBAT*4.69);
	PRINTF("VBat= %f V\n",battVoltage);
}

uint8_t Decode(uint8_t RxBuffer)
{
	/*
		*Decode the Telecommands
		*/
	uint8_t success=0;
	//PRINTF("Test");
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

uint8_t ReceiveBuffer(uint8_t RxBuffer)
{
	uint8_t success=0;

	switch (ReceiveState){

	case 0:
		SignFlag=0;
		DotFlag=0;
		DataIndex=0;
		if (RxBuffer==TelecommandStart)
		{
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
			PRINTF(ReceiveData);
			//success=Command(TelecommandID);
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
uint8_t Decode2(uint8_t RxBuffer)
{
	uint8_t success=0;
	switch (ReceiveState){
	case 0:
		SignFlag=0;
		DotFlag=0;
		DataIndex=0;
		//PRINTF("Test");
		if (RxBuffer==TelecommandStart)
		{
			//PRINTF("TCStart");
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

uint8_t Command(uint8_t TelecommandID)
{
	/*
	 * Execute the telecommands with respect to the telecommand ID
	 */
	char string[40];
	PRINTF("RECEIVED\n");
	UART_Pi.write("RECEIVED\n",9);

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
		//Enable/Disable Hbridge of the same unit
		HBridgeENStatus=int(atof(ReceiveData));
		PRINTF("RECEIVED %d \n",HBridgeENStatus);
		if(HBridgeENStatus==-1)
		{
			//Disable the HBridge
			PRINTF("DISABLING EPS1 \n ");
			//Set the PWM to 0 before Disabling/Enabling the HBridge, Recommended from the datasheet
			setHBridge(1,0);
			setHBridge(2,0);
			setHBridge(3,0);
			setHBridge(4,0);
			//AT(10*SECONDS);
			setENStatus(EPS1_HBridge_EN,0);
		}
		else
			setENStatus(EPS1_HBridge_EN,1);

		return 1;
	case Unit2EN:
		//Enable/Disable the second unit
		Unit2ENStatus=int(atof(ReceiveData));
		PRINTF("RECEIVED %d \n",Unit2ENStatus);
		if(Unit2ENStatus==-1)
		{
			PRINTF("DISABLING EPS2 Unit\n ");
			setENStatus(EPS2_EN,0);
		}
		else
			setENStatus(EPS2_EN,1);
		return 1;
	case BATCHGEN:
		//Enable for Disable the Battery charging
		BatCHGENStatus=int(atof(ReceiveData));
		PRINTF("RECEIVED %d \n",BatCHGENStatus);
		if(BatCHGENStatus==1)
		{
			PRINTF("DISABLING Battery Charging\n ");
			setENStatus(CHG_EN,0);
		}
		else
			setENStatus(CHG_EN,1);
		return 1;
	case DeployCMD:
		//Command to deploy the antenna for given time in seconds
		DeploymentTime=int(atof(ReceiveData));
		PRINTF("RECEIVED %d \n",DeploymentTime);
		thermalKnifeOON(Deployment_IN,DeploymentTime);
		return 1;
	case HBridge1PWM:
		//Sets the PWM for HBridge 1 (EM1)
		HBridgeValue=float(atof(ReceiveData));
		setHBridge(1,HBridgeValue);
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case HBridge2PWM:
		//Sets the PWM for HBridge 2 (EM2)
		HBridgeValue=float(atof(ReceiveData));
		setHBridge(2,HBridgeValue);
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case HBridge3PWM:
		//Sets the PWM for HBridge 3 (EM3)
		HBridgeValue=float(atof(ReceiveData));
		setHBridge(3,HBridgeValue);
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case HBridge4PWM:
		//Sets the PWM for HBridge 3 (EM3)
		HBridgeValue=float(atof(ReceiveData));
		setHBridge(4,HBridgeValue);
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case TakeImage:
		//Sends command to raspberry pi to take an image, just for the testing
			PRINTF("RECEIVED to Take Image\n");
			UART_Pi.write("$$TAM,CAM\n",10); ///Send this command to pi to take an image
			return 1;
	default:
		return 0;
	}
}

class DockingMain: public Thread {
public:
	void init()
	{
		//Initialize the LEDs
		LED0.init(true,1,0);
		LED1.init(true,1,0);
		LED2.init(true,1,0);
		LED3.init(true,1,0);
		LED4.init(true,1,0);
		LED5.init(true,1,0);
		LED6.init(true,1,0);
		LED7.init(true,1,0);


		EPS1_HBridge_EN.init(true,1,0); //Initially disable the HBridge of the unit 1

		EPS1_EN.init(true, 1, 1); //Initially enable the EPS of the unit 1, Own EPS

		CHG_EN.init(true,1, 0);  //Initially enable the Battery Charging

		EM_ADC.init(OCC1_CH); //Initialize the ADCs for the current measurement
		EM_ADC.init(OCC2_CH);
		EM_ADC.init(OCC3_CH);
		EM_ADC.init(OCC4_CH);


		BATT_ADC.config(ADC_PARAMETER_RESOLUTION,12); //Config and Initialize the ADC for the battery voltage measurement
		BATT_ADC.init(BATT_MES_ADC_CH);

		/*
		 *For old version with 2 units
		if (UNIT_PRIMARY)
			EPS2_EN.init(true, 1, 0); //Initially enable the EPS of the unit 1
		else
			EPS2_EN.init(true, 1, 1);
			*/

		//EPS1_HBridge_EN.init(true,1,1); //Initially enable the HBridge of the unit 1

		Deployment_IN.init(true,1,0); //Initialize the thermal knife and turn it off by default

		//Initialize HBrdige Input Pins
		HBridge1_IN1.init(true,1,0);
		HBridge1_IN2.init(true,1,0);
		HBridge2_IN1.init(true,1,0);
		HBridge2_IN2.init(true,1,0);
		HBridge3_IN1.init(true,1,0);
		HBridge3_IN2.init(true,1,0);
		HBridge4_IN1.init(true,1,0);
		HBridge4_IN2.init(true,1,0);

		//Initialize the PWM pins for the electromagnets
		EM_PWM1.init(10000,1000);
		EM_PWM2.init(10000,1000);
		EM_PWM3.init(10000,1000);
		EM_PWM4.init(10000,1000);
	}
	void run()
	{
		PRINTF("Project Name: TAMARIW \n");
		PRINTF("Program: TAMARIW EPS Testings \n");
		PRINTF(" LEHRSTUHL FOR INFORMATIK VIII \n");
		/*
		if (UNIT_PRIMARY)
			setENStatus(EPS2_EN,0); //Initially enable the EPS of the unit 1
		else
		{
			//Only disable after 5 seconds for secondary unit.
			//If the primary unit is not switched on within first 5 seconds, it secondary will turn the first unit OFF
			PRINTF("Secondary EPS Starting\n");
			AT(5*SECONDS);
			setENStatus(EPS2_EN,0);
			PRINTF("Secondary EPS Started\n");
		}
		*/
		 while(1){

			 /* Testing LEDs Pins*/
			 	LED0.setPins(1);
			 	LED1.setPins(1);
				LED2.setPins(1);
				LED3.setPins(1);
			 	LED4.setPins(1);
			 	LED5.setPins(1);
				LED6.setPins(1);
				LED7.setPins(1);
				suspendCallerUntil(NOW()+1000*MILLISECONDS);
			 	LED0.setPins(0);
				LED1.setPins(0);
				LED2.setPins(0);
				LED3.setPins(0);
			 	LED4.setPins(0);
			 	LED5.setPins(0);
				LED6.setPins(0);
				LED7.setPins(0);
				suspendCallerUntil(NOW()+1000*MILLISECONDS);

				SensorsTelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);
				LidarDataBuffer.getOnlyIfNewData(LidarDataReceiver);

				getEMCurrent(); //Get and print the electromagnets current
				PRINTF("Distance Main=%f \n",TelecommandDataReceiver.DistanceUWB);
				PRINTF("Lidar D1=%d ,",LidarDataReceiver.lidar1);
				PRINTF("D2=%d ,",LidarDataReceiver.lidar2);
				PRINTF("D3=%d ,",LidarDataReceiver.lidar3);
				PRINTF("D4=%d \n",LidarDataReceiver.lidar4);

				getBATVoltage(); //Get amd print the battery voltages

				PRINTF("\n");
				/*char floatVal[10];
				memcpy(&floatVal, &TelecommandDataReceiver.DistanceUWB, 10);
				UART_Pi.write("DIST=",10);
				float dist=TelecommandDataReceiver.DistanceUWB;
				*/
				/*char uart_data[100];
				sprintf(uart_data,"Distance Main=%f \n",TelecommandDataReceiver.DistanceUWB);
				UART_Pi.write(uart_data,strlen(uart_data));
*/

		 }
		//EPS1_HBridge_EN.setPins(1);

	}
} DockingEPS_obj;




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


class UARTClassPi: public Thread {
	uint64_t periode;
	//HAL_UART *uart;
public:

	UARTClassPi(const char* name, uint64_t periode) : Thread(name) {
		this->periode = periode;
		//this->uart = uart;
	}

	void init() {
		//uart->init(115200);
		UART_Pi.init(115200);
		//UART_Pi.config(UART_PARAMETER_ENABLE_DMA, 1);
	}

	void run(){
		char RxBuffer;
		const char test[]="Testing Pi Serial \n\r";
		write(test,sizeof(test)-1);
		while(1)
		{
			UART_Pi.suspendUntilDataReady();
			UART_Pi.read(&RxBuffer,1);
			Decode(RxBuffer);
			//ReceiveBuffer(RxBuffer);
		}

/*
		TIME_LOOP(0,periode){
			write(test,sizeof(test)-1);
			//UART_Pi.suspendUntilDataReady();
            //UART_Pi.read(&RxBuffer,1);
            //Decode2(RxBuffer);
		}
*/

	}
	void write(const char *buf, int size){
	    int sentBytes=0;
	    int retVal;
	    while(sentBytes < size){
	        retVal = UART_Pi.write(&buf[sentBytes],size-sentBytes);
	        if (retVal < 0){
	            PRINTF("UART sent error\n");
	        }else{
	            sentBytes+=retVal;
	        }
	    }
	}

};

UARTClassPi UARTPi("UARTPi", 100 * MILLISECONDS);

