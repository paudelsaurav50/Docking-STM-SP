#include "tcmd.h"
#include "topics.h"

HAL_UART serial(UART_IDX3, GPIO_026, GPIO_027);

// Parse the incoming message
bool thread_tcmd::parse(const char *msg, tcmd_t *tcmd)
{
  // Check if the message starts with TCMD_START_CHAR and ends with TCMD_STOP_CHAR
  if (msg[0] != TCMD_START_CHAR || msg[strlen(msg) - 1] != TCMD_STOP_CHAR)
  {
    return false; // Invalid message format
  }

  // Find the position of the delimiter
  const char *delimiter_pos = strchr(msg, TCMD_DELIMITER);
  if (!delimiter_pos)
  {
    return false; // Delimiter not found
  }

  // Extract and validate the command
  int command_int = 0;
  const char *ptr = msg + 1; // Start after TCMD_START_CHAR
  while (ptr < delimiter_pos)
  {
    if (*ptr >= '0' && *ptr <= '9')
    {
      command_int = command_int * 10 + (*ptr - '0'); // Convert char to int
    }
    else
    {
      return false; // Invalid character in command
    }
    ptr++;
  }

  // Validate the command range using TCMD_LENGTH
  if (command_int < 0 || command_int >= TCMD_LENGTH)
  {
    return false; // Invalid command
  }

  // Extract and convert the data
  float data_value = 0.0f;
  ptr = delimiter_pos + 1; // Start after TCMD_DELIMITER
  bool decimal_found = false;
  float decimal_place = 0.1f;

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
      decimal_found = true;
    }
    else // Invalid character in data
    {
      return false;
    }
    ptr++;
  }

  // Assign the parsed values
  tcmd->idx = static_cast<tcmd_idx>(command_int);
  tcmd->data = data_value;

  return true; // Success
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

        PRINTF("Successful telecommand reception!\n");
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
    rx.stop = false;
    rx.i[0] = cmd->data;
    break;

  case TCMD_EM1:
    rx.stop = false;
    rx.i[1] = cmd->data;
    break;

  case TCMD_EM2:
    rx.stop = false;
    rx.i[2] = cmd->data;
    break;

  case TCMD_EM3:
    rx.stop = false;
    rx.i[3] = cmd->data;
    break;

  case TCMD_EM0_STOP:
    rx.i[0] = 0.0;
    break;

  case TCMD_EM1_STOP:
    rx.i[1] = 0.0;
    break;

  case TCMD_EM2_STOP:
    rx.i[2] = 0.0;
    break;

  case TCMD_EM3_STOP:
    rx.i[3] = 0.0;
    break;

  case TCMD_EM_STOP_ALL:
    rx.i[0] = 0.0;
    rx.i[1] = 0.0;
    rx.i[2] = 0.0;
    rx.i[3] = 0.0;
    rx.stop = true;

  default:
    break;
  }

  return true;
}

static thread_tcmd tcmd("tcmd", TCMD_THREAD_PROIRITY);
Topic<tcmd_t> topic_tcmd(-1, "tcmd");
