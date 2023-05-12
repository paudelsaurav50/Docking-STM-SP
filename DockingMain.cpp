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
#define DEP_IN GPIO_043 //PC11, Thermal Knife

#define PWM4_2 PWM_IDX00 //Timer 1 CH1 - PE9, HBridge 4 input 2
#define PWM4_1 PWM_IDX01 //Timer 1 CH2 - PE11, HBridge 4 input 1
#define PWM3_2 PWM_IDX02 //Timer 1 CH3 - PE13, HBridge 3 input 2
#define PWM3_1 PWM_IDX03 //Timer 1 CH4 - PE14, HBridge 3 input 1
#define PWM2_2 PWM_IDX12 //Timer 4 CH1 - PD12, HBridge 2 input 2
#define PWM2_1 PWM_IDX13 //Timer 4 CH2 - PD13, HBridge 2 input 1
#define PWM1_2 PWM_IDX14 //Timer 4 CH3 - PD14, HBridge 1 input 2
#define PWM1_1 PWM_IDX15 //Timer 4 CH4 - PD15, HBridge 1 input 1

#define OCC1_CH ADC_CH_010 //ADC3_CH10, PC0, HBridge1 current monitor
#define OCC2_CH ADC_CH_011 //ADC3_CH11, PC1, HBridge2 current monitor
#define OCC3_CH ADC_CH_012 //ADC3_CH12, PC2, HBridge3 current monitor
#define OCC4_CH ADC_CH_013 //ADC3_CH13, PC3, HBridge4 current monitor
#define ADC_NO ADC_IDX3 //Using ADC 3 for PC0,1,2 and 3



HAL_GPIO EPS1_EN(EN_EPS_1);
HAL_GPIO EPS2_EN(EN_EPS_2);
HAL_GPIO EPS1_HBridge_EN(EN_HBRIDGE_1);
HAL_GPIO Deployment_IN(DEP_IN);

HAL_GPIO LED0(GPIO_048);
HAL_GPIO LED1(GPIO_049);
HAL_GPIO LED2(GPIO_050);
HAL_GPIO LED3(GPIO_051);
HAL_GPIO LED4(GPIO_052);
HAL_GPIO LED5(GPIO_053);
HAL_GPIO LED6(GPIO_054);
HAL_GPIO LED7(GPIO_055);

HAL_PWM  HBridge1_IN1(PWM1_1);
HAL_PWM  HBridge1_IN2(PWM1_2);
HAL_PWM  HBridge2_IN1(PWM2_1);
HAL_PWM  HBridge2_IN2(PWM2_2);
HAL_PWM  HBridge3_IN1(PWM3_1);
HAL_PWM  HBridge3_IN2(PWM3_2);
HAL_PWM  HBridge4_IN1(PWM4_1);
HAL_PWM  HBridge4_IN2(PWM4_2);


HAL_UART UART_Pi(UART_IDX3, GPIO_026, GPIO_027);

HAL_ADC EM_ADC(ADC_NO);


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

			/*pin.setPins(1);

			*/
			diffTime=(int)(SECONDS_NOW()-initTme);
		}

		//pin.setPins(1);
		//AT(Time_s*SECONDS);
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
				setPWM(HBridge1_IN1,-1*Value);
				setPWM(HBridge1_IN2,0);
			}
			else
			{
				setPWM(HBridge1_IN1,0);
				setPWM(HBridge1_IN2,1*Value);
			}
			break;
		case 2:
			//RUN 2nd HBridge
			if(Value<0)
			{
				setPWM(HBridge2_IN1,-1*Value);
				setPWM(HBridge2_IN2,0);
			}
			else
			{
				setPWM(HBridge2_IN1,0);
				setPWM(HBridge2_IN2,1*Value);
			}
			break;
		case 3:
			//RUN 3rd HBridge
			if(Value<0)
			{
				setPWM(HBridge3_IN1,-1*Value);
				setPWM(HBridge3_IN2,0);
			}
			else
			{
				setPWM(HBridge3_IN1,0);
				setPWM(HBridge3_IN2,1*Value);
			}
			break;
		case 4:
			//RUN 4th HBridge
			if(Value<0)
			{
				setPWM(HBridge4_IN1,-1*Value);
				setPWM(HBridge4_IN2,0);
			}
			else
			{
				setPWM(HBridge4_IN1,0);
				setPWM(HBridge4_IN2,1*Value);
			}
			break;

		}
}


void getEMCurrent()
{
	/*
	 *@Output: Current through Electromagnets
		*/
	float adcValOCM1 = ((float(EM_ADC.read(OCC1_CH)))/4096)*3290;
	float adcValOCM2 = ((float(EM_ADC.read(OCC2_CH)))/4096)*3290;
	float adcValOCM3 = ((float(EM_ADC.read(OCC3_CH)))/4096)*3290;
	float adcValOCM4 = ((float(EM_ADC.read(OCC4_CH)))/4096)*3290;

	float currentEM1=(3/1.47)*(adcValOCM1/1000);
	float currentEM2=(3/1.47)*(adcValOCM2/1000);
	float currentEM3=(3/1.47)*(adcValOCM3/1000);
	float currentEM4=(3/1.47)*(adcValOCM4/1000);
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

uint8_t Decode(uint8_t RxBuffer)
{
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
	char string[40];
	PRINTF("RECEIVED\n");
	UART_Pi.write("RECEIVED\n",9);

	float a;
	int HBridgeENStatus;
	int Unit2ENStatus;
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
	case DeployCMD:
		DeploymentTime=int(atof(ReceiveData));
		PRINTF("RECEIVED %d \n",DeploymentTime);
		thermalKnifeOON(Deployment_IN,DeploymentTime);
		return 1;
	case HBridge1PWM:
		HBridgeValue=float(atof(ReceiveData));
		setHBridge(1,HBridgeValue);
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case HBridge2PWM:
		HBridgeValue=float(atof(ReceiveData));
		setHBridge(2,HBridgeValue);
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case HBridge3PWM:
		HBridgeValue=float(atof(ReceiveData));
		setHBridge(3,HBridgeValue);
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case HBridge4PWM:
		HBridgeValue=float(atof(ReceiveData));
		setHBridge(4,HBridgeValue);
		PRINTF("RECEIVED %f \n",HBridgeValue);
		return 1;
	case TakeImage:
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
		LED0.init(true,1,0);
		LED1.init(true,1,0);
		LED2.init(true,1,1);
		LED3.init(true,1,1);
		LED4.init(true,1,0);
		LED5.init(true,1,0);
		LED6.init(true,1,1);
		LED7.init(true,1,1);
		/*
		EPS1_HBridge_EN.init(true,1,0); //Initially disable the HBridge of the unit 1

		EPS1_EN.init(true, 1, 1); //Initially enable the EPS of the unit 1, Own EPS

		EM_ADC.init(OCC1_CH); //Initialize the ADCs for voltage measurement for the current measurement
		EM_ADC.init(OCC2_CH);
		EM_ADC.init(OCC3_CH);
		EM_ADC.init(OCC4_CH);

		if (UNIT_PRIMARY)
			EPS2_EN.init(true, 1, 0); //Initially enable the EPS of the unit 1 //Initially enable the EPS of the unit 1
		else
			EPS2_EN.init(true, 1, 1);

		//EPS1_HBridge_EN.init(true,1,1); //Initially enable the HBridge of the unit 1

		Deployment_IN.init(true,1,0); //initially disable the deployment pin

		//Initialize all the Pins
		HBridge1_IN1.init(1000,1000);
		HBridge1_IN2.init(1000,1000);
		HBridge2_IN1.init(1000,1000);
		HBridge2_IN2.init(1000,1000);
		HBridge3_IN1.init(1000,1000);
		HBridge3_IN2.init(1000,1000);
		HBridge4_IN1.init(1000,1000);
		HBridge4_IN2.init(1000,1000);
		*/

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
			 	//LED6.setPins(1);
			 	//LED7.setPins(0);
				LED2.setPins(1);
				LED3.setPins(1);
				LED6.setPins(1);
				LED7.setPins(1);
				suspendCallerUntil(NOW()+2000*MILLISECONDS);
				LED2.setPins(0);
				LED3.setPins(0);
				LED6.setPins(0);
				LED7.setPins(0);
				suspendCallerUntil(NOW()+2000*MILLISECONDS);
				/*
				SensorsTelecommandDataBuffer.getOnlyIfNewData(TelecommandDataReceiver);
				LidarDataBuffer.getOnlyIfNewData(LidarDataReceiver);
				getEMCurrent();
				PRINTF("Distance Main=%f \n",TelecommandDataReceiver.DistanceUWB);
				PRINTF("Lidar Main D1=%d \n",LidarDataReceiver.lidar1);
				*/
				/*char floatVal[10];
				memcpy(&floatVal, &TelecommandDataReceiver.DistanceUWB, 10);
				UART_Pi.write("DIST=",10);
				float dist=TelecommandDataReceiver.DistanceUWB;
				*/
				char uart_data[100];
				sprintf(uart_data,"Distance Main=%f \n",TelecommandDataReceiver.DistanceUWB);
				UART_Pi.write(uart_data,strlen(uart_data));


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

