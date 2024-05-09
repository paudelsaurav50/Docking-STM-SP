
#include <stdlib.h>
#include "magnet.h"
#include "telecommand.h"
#include "satellite_config.h"

float desired_current[4] = {0.0};

namespace RODOS
{
  extern HAL_UART uart_stdout;
}

#define TeleUART uart_stdout

uint8_t ReceiveState = 0;
uint8_t SignFlag = 0;
uint8_t  DotFlag = 0;
uint8_t DataIndex = 0;
char telecommand_id;
char ReceiveData[MaxLength];

uint8_t decode_command(uint8_t rx_buffer)
{
  uint8_t success=0;

  switch (ReceiveState)
  {
  case 0:
    SignFlag=0;
    DotFlag=0;
    DataIndex=0;
    if (rx_buffer==TelecommandStart)
    {
      ReceiveState=1;
    }
    break;

  case 1:
    SignFlag=0;
    DotFlag=0;
    DataIndex=0;
    if (rx_buffer==TelecommandStart)
    {
      ReceiveState=1;
    }
    else {
      telecommand_id = rx_buffer;
      ReceiveState = 2;
    }
    break;

  case 2:
    if (rx_buffer=='+' || rx_buffer=='-')
    {
      if (SignFlag==0 && DataIndex==0)
        {
        SignFlag=1;
        ReceiveData[DataIndex]=rx_buffer;
        DataIndex++;
        ReceiveState = 2;
        }
      else {ReceiveState = 0;}
    }
    else if (rx_buffer=='.')
    {
      if (DotFlag==0)
        {
          DotFlag=1;
          ReceiveData[DataIndex]=rx_buffer;
          DataIndex++;
          ReceiveState = 2;
        }
      else {ReceiveState = 0;}
    }
    else if (rx_buffer>='0' && rx_buffer<='9')
    {
        ReceiveData[DataIndex]=rx_buffer;
        DataIndex++;
      if (DataIndex > MaxLength) {ReceiveState = 0;}
      else {ReceiveState = 2;}
    }
    else if (rx_buffer==TelecommandStart)
    {
      ReceiveState=1;
    }
    else if (rx_buffer==TelecommandStop)
    {
      ReceiveData[DataIndex]= 0x00;
      success=execute_command(telecommand_id);
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

uint8_t execute_command(uint8_t telecommand_id)
{
  switch (telecommand_id)
  {
  case HBridgeUnit1EN:
  {
    if(int(atof(ReceiveData))==-1)
    {
      magnet::actuate(MAGNET_IDX_ALL, 0);
    }
    break;
  }
  case HBridge1PWM:
  {
    magnet::actuate(MAGNET_IDX_0, 50);
    break;
  }
  case HBridge2PWM:
  {
    magnet::actuate(MAGNET_IDX_1, 50);
    break;
  }
  case HBridge3PWM:
  {
    magnet::actuate(MAGNET_IDX_2, 50);
    break;
  }
  case HBridge4PWM:
  {
    magnet::actuate(MAGNET_IDX_3, 50);
    break;
  }
  default:
  {
    return 0;
  }
  }

  return 1;
}

void telecommand_thread::init()
{
  magnet::init();
}

void telecommand_thread::run()
{
  char rx_buffer;

  while (1)
  {
    TeleUART.suspendUntilDataReady();
    TeleUART.read(&rx_buffer,1);
    decode_command(rx_buffer);
  }
}

telecommand_thread tamariw_telecommand_thread;
