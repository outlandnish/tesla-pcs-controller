/*
 * Error message handling for Arduino port
 * Based on libopeninv error message system
 */
#ifndef ERRORMESSAGE_H
#define ERRORMESSAGE_H

#include "errormessage_prj.h"
#include <stdint.h>

#define ERROR_MESSAGE_ENTRY(id, type) ERR_##id,
typedef enum
{
   ERROR_NONE,
   ERROR_MESSAGE_LIST
   ERROR_MESSAGE_LAST
} ERROR_MESSAGE_NUM;
#undef ERROR_MESSAGE_ENTRY

typedef enum
{
   ERROR_STOP,
   ERROR_DERATE,
   ERROR_DISPLAY,
   ERROR_LAST
} ERROR_TYPE;

struct ErrorDescriptor {
   const char* msg;
   ERROR_TYPE type;
};

extern const struct ErrorDescriptor errorDescriptors[];

class ErrorMessage
{
public:
   static void SetTime(uint32_t time);
   static void Post(ERROR_MESSAGE_NUM err);
   static void UnpostAll();
   static ERROR_MESSAGE_NUM GetLastError();
   static ERROR_MESSAGE_NUM GetErrorNum(uint8_t index);
   static uint32_t GetErrorTime(uint8_t index);

private:
   static uint32_t timeTick;
   static uint32_t currentBufIdx;
   static bool posted[ERROR_MESSAGE_LAST];
   static ERROR_MESSAGE_NUM lastError;
};

#endif // ERRORMESSAGE_H
