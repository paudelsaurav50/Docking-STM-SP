#include "tcmd.h"
#include "topics.h"

HAL_UART serial(UART_IDX3, GPIO_026, GPIO_027);

bool thread_tcmd::parse(const char *msg, tcmd_t *tcmd)
{
  if (msg[0] != TCMD_START_CHAR || msg[strlen(msg) - 1] != TCMD_STOP_CHAR)
  {
    return false; // Invalid message format
  }

  const char *delimiter_pos = strchr(msg, TCMD_DELIMITER);
  if (!delimiter_pos)
  {
    return false; // Delimiter not found
  }

  int command_int = 0;
  const char *ptr = msg + 1;
  while (ptr < delimiter_pos)
  {
    if (*ptr >= '0' && *ptr <= '9')
    {
      command_int = command_int * 10 + (*ptr - '0');
    }
    else
    {
      return false;
    }
    ptr++;
  }

  if (command_int < 0 || command_int >= TCMD_LENGTH)
  {
    return false;
  }

  // Begin parsing data
  float data_value = 0.0f;
  bool decimal_found = false;
  float decimal_place = 0.1f;
  int sign = 1;

  ptr = delimiter_pos + 1;

  // Negative sign
  if (*ptr == '-')
  {
    sign = -1;
    ptr++;
  }

  while (*ptr != TCMD_STOP_CHAR && *ptr != '\0')
  {
    if (*ptr >= '0' && *ptr <= '9')
    {
      if (decimal_found)
      {
        data_value += (*ptr - '0') * decimal_place;
        decimal_place *= 0.1f;
      }
      else
      {
        data_value = data_value * 10.0f + (*ptr - '0');
      }
    }
    else if (*ptr == '.')
    {
      // Two decimals
      if (decimal_found)
      {
        return false;
      }

      decimal_found = true;
    }
    else // Invalid character
    {
      return false;
    }
    ptr++;
  }

  tcmd->idx = static_cast<tcmd_idx>(command_int);
  tcmd->data = sign * data_value;

  return true;
}

void thread_tcmd::init()
{
  serial.init(115200);
}

void thread_tcmd::run()
{
  init();

  TIME_LOOP(0 * MILLISECONDS, period_ms * MILLISECONDS)
  {
    size_t rxlen = serial.read(tcmd_msg, sizeof(tcmd_msg));

    if (rxlen > 0)
    {
      tcmd_t tcmd;

      if (parse(tcmd_msg, &tcmd))
      {
        topic_tcmd.publish(tcmd);
        execute(&tcmd);

        PRINTF("Successful telecommand reception! %s\n", tcmd_msg);
        serial.write("Successful telecommand reception!\n", 35);
      }
      else
      {
        PRINTF("Invalid telecommand!\n");
      }
    }
  }
}

bool thread_tcmd::execute(const tcmd_t *cmd)
{
  switch (cmd->idx)
  {
  case TCMD_EM0:
    rx.stop_coils = false;
    rx.stop_coil[0] = false;
    rx.i[0] = cmd->data;
    break;

  case TCMD_EM1:
    rx.stop_coils = false;
    rx.stop_coil[1] = false;
    rx.i[1] = cmd->data;
    break;

  case TCMD_EM2:
    rx.stop_coils = false;
    rx.stop_coil[2] = false;
    rx.i[2] = cmd->data;
    break;

  case TCMD_EM3:
    rx.stop_coils = false;
    rx.stop_coil[3] = false;
    rx.i[3] = cmd->data;
    break;

  case TCMD_EM0_STOP:
    rx.stop_coil[0] = true;
    break;

  case TCMD_EM1_STOP:
    rx.stop_coil[1] = true;
    break;

  case TCMD_EM2_STOP:
    rx.stop_coil[2] = true;
    break;

  case TCMD_EM3_STOP:
    rx.stop_coil[3] = true;
    break;

  case TCMD_EM_STOP_ALL:
    rx.stop_coil[0] = true;
    rx.stop_coil[1] = true;
    rx.stop_coil[2] = true;
    rx.stop_coil[3] = true;
    rx.stop_coils = true;
    break;

  case TCMD_EM_KP:
    rx.kp = cmd->data;
    break;

  case TCMD_EM_KI:
    rx.ki = cmd->data;
    break;

  case TCMD_KF_Q00:
    rx.q_pos = cmd->data;
    break;

  case TCMD_KF_Q11:
    rx.q_vel = cmd->data;
    break;

  case TCMD_KF_R:
    rx.r = cmd->data;
    break;

  default:
    break;
  }

  return true;
}

static thread_tcmd tcmd("tcmd", TCMD_THREAD_PROIRITY);
Topic<tcmd_t> topic_tcmd(-1, "tcmd");
