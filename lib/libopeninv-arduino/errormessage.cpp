/*
 * Error message implementation for Arduino port
 * Based on libopeninv error message system
 */
#include "errormessage.h"
#include "debug_serial.h"

struct BufferEntry
{
   ERROR_MESSAGE_NUM msg;
   uint32_t time;
};

#define ERROR_MESSAGE_ENTRY(id, type) { #id, type },
const struct ErrorDescriptor errorDescriptors[] =
{
   { "", ERROR_LAST },
   ERROR_MESSAGE_LIST
};
#undef ERROR_MESSAGE_ENTRY

static const char* types[ERROR_LAST] =
{
   "STOP",
   "DERATE",
   "WARN"
};

static struct BufferEntry errorBuffer[ERROR_BUF_SIZE] = { { ERROR_MESSAGE_LAST, 0 } };

uint32_t ErrorMessage::timeTick = 0;
uint32_t ErrorMessage::currentBufIdx = 0;
ERROR_MESSAGE_NUM ErrorMessage::lastError = ERROR_NONE;
bool ErrorMessage::posted[ERROR_MESSAGE_LAST] = { false };

/** Set timestamp for error message
* @param time Current timestamp, will be displayed as is in message */
void ErrorMessage::SetTime(uint32_t time)
{
   timeTick = time;
}

/** Post an error message.
 Every message can only be posted once, then UnpostAll() must be called to post it again
 @post Message is displayed and written to error memory
 @param msg message number */
void ErrorMessage::Post(ERROR_MESSAGE_NUM msg)
{
   if (!posted[msg] && timeTick > 0 && msg < ERROR_MESSAGE_LAST)
   {
      lastError = msg;
      errorBuffer[currentBufIdx].msg = msg;
      errorBuffer[currentBufIdx].time = timeTick;
      posted[msg] = true;
      
      #ifdef ARDUINO
      DEBUG_SERIAL.printf("[%u] ERROR: %s\r\n", 
                         timeTick,  
                         errorDescriptors[msg].msg);
      #endif
      
      currentBufIdx = (currentBufIdx + 1) % ERROR_BUF_SIZE;
   }
}

/** Unpost all error messages, i.e. make them postable again.
 Does not reset the error buffer */
void ErrorMessage::UnpostAll()
{
   for (uint32_t i = 0; i < ERROR_MESSAGE_LAST; i++)
      posted[i] = false;
}

ERROR_MESSAGE_NUM ErrorMessage::GetLastError()
{
   return lastError;
}

ERROR_MESSAGE_NUM ErrorMessage::GetErrorNum(uint8_t index)
{
   if (index < ERROR_BUF_SIZE)
   {
      if (errorBuffer[index].time > 0)
         return errorBuffer[index].msg;
   }

   return ERROR_NONE;
}

uint32_t ErrorMessage::GetErrorTime(uint8_t index)
{
   if (index < ERROR_BUF_SIZE)
   {
      return errorBuffer[index].time;
   }

   return 0;
}
