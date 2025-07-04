#include "tcmd.h"
#include "topics.h"
#include "sat_config.h"

HAL_UART serial(UART_IDX3, GPIO_026, GPIO_027);

bool thread_tcmd::parse(const char *msg, tcmd_t *tcmd)
{
  // PRINTF("%s\n", msg);

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
  timekeeper = NOW();
  period_ms = THREAD_PERIOD_TCMD_MILLIS;

  serial.init(115200);
}

void thread_tcmd::run()
{
  init();

  TIME_LOOP(THREAD_START_TCMD_MILLIS * MILLISECONDS, period_ms * MILLISECONDS)
  {
    memset(tcmd_msg, 0, sizeof(tcmd_msg));
    size_t rxlen = serial.read(tcmd_msg, sizeof(tcmd_msg));

    if (rxlen > 0)
    {
      tcmd_t tcmd;

      if (parse(tcmd_msg, &tcmd))
      {
        topic_tcmd.publish(tcmd);

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
  return true;
}

thread_tcmd tcmd("tcmd", THREAD_PRIO_TCMD);
