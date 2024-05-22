
#include <stdlib.h>
#include "magnet.h"
#include "telecommand.h"
#include "satellite_config.h"
#include "collision_control.h"

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
char ReceiveData[TELECOMMAND_MAX_LEN];

uint8_t decode_command(uint8_t rx_buffer)
{
  uint8_t success=0;

  switch (ReceiveState)
  {
  case 0:
    SignFlag=0;
    DotFlag=0;
    DataIndex=0;
    if (rx_buffer==TELECOMMAND_START)
    {
      ReceiveState=1;
    }
    break;

  case 1:
    SignFlag=0;
    DotFlag=0;
    DataIndex=0;
    if (rx_buffer==TELECOMMAND_START)
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
      if (DataIndex > TELECOMMAND_MAX_LEN) {ReceiveState = 0;}
      else {ReceiveState = 2;}
    }
    else if (rx_buffer==TELECOMMAND_START)
    {
      ReceiveState=1;
    }
    else if (rx_buffer==TELECOMMAND_STOP)
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
  case ENABLE_CONTROL:
  {
    if(int(atof(ReceiveData))== 1) // Idle mode
    {
      // Magnets off and disable magnet thread
      desired_current[0] = 0;
      desired_current[1] = 0;
      desired_current[2] = 0;
      desired_current[3] = 0;
      tamariw_collision_control_thread.stop_thread = true;
    }
    else // Resume control thread
    {
      tamariw_collision_control_thread.stop_thread = false;
      tamariw_collision_control_thread.resume();
    }
    break;
  }
  case TEST_MAGNETS:
  {
    desired_current[0] = 1000;
    desired_current[1] = 1000;
    desired_current[2] = 1000;
    desired_current[3] = 1000;
    break;
  }
  case PI_POS_GAIN_KP:
  {
    pid_distance.kp = float(atof(ReceiveData));
    break;
  }
  case PI_POS_GAIN_KI:
  {
    pid_distance.ki = float(atof(ReceiveData));
    break;
  }
  case PI_VEL_GAIN_KP:
  {
    pid_velocity.kp = float(atof(ReceiveData));
    break;
  }
  case PI_VEL_GAIN_KI:
  {
    pid_velocity.ki = float(atof(ReceiveData));
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

telecommand_thread tamariw_telecommand_thread("telecommand_thread");
