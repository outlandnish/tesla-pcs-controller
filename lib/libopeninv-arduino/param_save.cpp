/*
 * Arduino port of libopeninv param_save
 * Uses STM32 HAL flash functions instead of libopencm3
 */

#include <Arduino.h>
#include "stm32f4xx_hal.h"
#include "params.h"
#include "param_save.h"
#include "my_string.h"
#include "my_fp.h"
#include "debug_serial.h"

// Flash configuration - save to last flash page
#ifndef PARAM_BLKSIZE
#define PARAM_BLKSIZE 2048  // Flash page size for STM32F4
#endif

#ifndef PARAM_BLKNUM
#define PARAM_BLKNUM 2      // Use second-to-last flash page (avoids 64KB boundary at 0x08180000)
#endif

#define NUM_PARAMS ((PARAM_BLKSIZE - 8) / sizeof(PARAM_ENTRY))
#define PARAM_WORDS (PARAM_BLKSIZE / 4)

typedef struct __attribute__((packed))
{
   uint16_t key;
   uint8_t dummy;
   uint8_t flags;
   uint32_t value;
} PARAM_ENTRY;

typedef struct __attribute__((packed))
{
   PARAM_ENTRY data[NUM_PARAMS];
   uint32_t crc;
   uint32_t padding;
} PARAM_PAGE;

// Simple CRC32 calculation
static uint32_t calculate_crc32(uint32_t *data, uint32_t length)
{
   uint32_t crc = 0xFFFFFFFF;

   for (uint32_t i = 0; i < length; i++)
   {
      crc ^= data[i];
      for (int j = 0; j < 32; j++)
      {
         if (crc & 1)
            crc = (crc >> 1) ^ 0xEDB88320;
         else
            crc = crc >> 1;
      }
   }

   return ~crc;
}

static uint32_t GetFlashAddress()
{
   // Get flash size from device signature
   uint16_t flashSize = *((uint16_t*)0x1FFF7A22); // Flash size in KB

   // Calculate address of parameter storage area
   uint32_t address = FLASH_BASE + (flashSize * 1024) - (PARAM_BLKNUM * PARAM_BLKSIZE);
   
   #ifdef DEBUG_PARAM_SAVE
   DEBUG_SERIAL.printf("GetFlashAddress: Flash size = %d KB\r\n", flashSize);
   DEBUG_SERIAL.printf("GetFlashAddress: FLASH_BASE = 0x%08X\r\n", FLASH_BASE);
   DEBUG_SERIAL.printf("GetFlashAddress: Parameter address = 0x%08X\r\n", address);
   #endif
   
   return address;
}

static uint32_t GetFlashSector(uint32_t address)
{
   // Determine which flash sector the address belongs to
   // STM32F4 has variable sector sizes
   if (address < 0x08004000) return FLASH_SECTOR_0;
   if (address < 0x08008000) return FLASH_SECTOR_1;
   if (address < 0x0800C000) return FLASH_SECTOR_2;
   if (address < 0x08010000) return FLASH_SECTOR_3;
   if (address < 0x08020000) return FLASH_SECTOR_4;
   if (address < 0x08040000) return FLASH_SECTOR_5;
   if (address < 0x08060000) return FLASH_SECTOR_6;
   if (address < 0x08080000) return FLASH_SECTOR_7;
   #ifdef FLASH_SECTOR_8
   if (address < 0x080A0000) return FLASH_SECTOR_8;
   if (address < 0x080C0000) return FLASH_SECTOR_9;
   if (address < 0x080E0000) return FLASH_SECTOR_10;
   return FLASH_SECTOR_11;
   #else
   return FLASH_SECTOR_7;
   #endif
}

/**
* Save parameters to flash
*
* @return CRC of parameter flash page
*/
uint32_t parm_save()
{
   PARAM_PAGE parmPage;
   uint32_t idx;
   uint32_t paramAddress = GetFlashAddress();
   uint32_t sector = GetFlashSector(paramAddress);

   DEBUG_SERIAL.printf("parm_save: sizeof(PARAM_ENTRY) = %d bytes\r\n", sizeof(PARAM_ENTRY));
   DEBUG_SERIAL.printf("parm_save: sizeof(PARAM_PAGE) = %d bytes\r\n", sizeof(PARAM_PAGE));
   DEBUG_SERIAL.printf("parm_save: NUM_PARAMS = %d\r\n", NUM_PARAMS);
   DEBUG_SERIAL.printf("parm_save: PARAM_BLKSIZE = %d bytes\r\n", PARAM_BLKSIZE);
   DEBUG_SERIAL.printf("parm_save: Data array size = %d bytes (%d words)\r\n",
                 sizeof(parmPage.data), sizeof(parmPage.data) / sizeof(uint32_t));

   memset32((int*)&parmPage, 0xFFFFFFFF, PARAM_WORDS);

   // Copy parameter values and keys to block structure
   DEBUG_SERIAL.println("parm_save: Writing parameters to flash block:");
   int saved = 0;
   for (idx = 0; idx < NUM_PARAMS && idx < Param::PARAM_LAST; idx++)
   {
      if (Param::GetType((Param::PARAM_NUM)idx) == Param::TYPE_PARAM)
      {
         const Param::Attributes *pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);
         parmPage.data[idx].flags = (uint8_t)Param::GetFlag((Param::PARAM_NUM)idx);
         parmPage.data[idx].key = pAtr->id;
         // Convert float to fixed-point for storage (SetFixed expects fixed-point on load)
         float value = Param::Get((Param::PARAM_NUM)idx);
         parmPage.data[idx].value = FP_FROMFLT(value);
         DEBUG_SERIAL.printf("  [%d] %s (id=%d): %.2f (0x%08X fixed)\r\n", 
                             idx, pAtr->name, pAtr->id, value, parmPage.data[idx].value);
         saved++;
      }
   }
   DEBUG_SERIAL.printf("parm_save: Prepared %d parameters for storage\r\n", saved);

   parmPage.crc = calculate_crc32((uint32_t*)&parmPage, 2 * NUM_PARAMS);

   DEBUG_SERIAL.printf("parm_save: Saving parameters to flash at 0x%08X (sector %d)\r\n", paramAddress, sector);
   DEBUG_SERIAL.printf("parm_save: Parameter block size: %d bytes (%d words)\r\n", PARAM_BLKSIZE, PARAM_WORDS);

   // Unlock flash
   HAL_StatusTypeDef status = HAL_FLASH_Unlock();
   if (status != HAL_OK)
   {
      DEBUG_SERIAL.printf("ERROR: Failed to unlock flash (status=%d)\r\n", status);
      return 0;
   }
   DEBUG_SERIAL.println("parm_save: Flash unlocked");

   // Clear any previous flash errors
   __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                          FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

   // Erase sector
   FLASH_EraseInitTypeDef eraseInit;
   uint32_t sectorError;

   eraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
   eraseInit.Sector = sector;
   eraseInit.NbSectors = 1;
   eraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3; // 2.7V to 3.6V

   DEBUG_SERIAL.printf("parm_save: Erasing sector %d...\r\n", sector);
   status = HAL_FLASHEx_Erase(&eraseInit, &sectorError);
   if (status != HAL_OK)
   {
      DEBUG_SERIAL.printf("ERROR: Failed to erase sector (status=%d, sectorError=0x%08X)\r\n", status, sectorError);
      HAL_FLASH_Lock();
      return 0;
   }
   DEBUG_SERIAL.println("parm_save: Sector erased successfully");

   // Program flash
   DEBUG_SERIAL.printf("parm_save: Programming %d words...\r\n", PARAM_WORDS);
   for (idx = 0; idx < PARAM_WORDS; idx++)
   {
      uint32_t* pData = ((uint32_t*)&parmPage) + idx;
      uint32_t addr = paramAddress + idx * sizeof(uint32_t);
      status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr, *pData);
      if (status != HAL_OK)
      {
         DEBUG_SERIAL.printf("ERROR: Failed to program flash at 0x%08X (status=%d)\r\n", addr, status);
         HAL_FLASH_Lock();
         return 0;
      }
   }
   DEBUG_SERIAL.println("parm_save: Flash programmed successfully");

   // Lock flash
   HAL_FLASH_Lock();
   DEBUG_SERIAL.println("parm_save: Flash locked, parameters saved successfully");
   DEBUG_SERIAL.printf("parm_save: CRC32 = 0x%08X\r\n", parmPage.crc);

   return parmPage.crc;
}

/**
* Load parameters from flash
*
* @retval 0 Parameters loaded successfully
* @retval -1 CRC error, parameters not loaded
*/
int parm_load()
{
   uint32_t paramAddress = GetFlashAddress();
   PARAM_PAGE *parmPage = (PARAM_PAGE *)paramAddress;

   uint32_t crc = calculate_crc32((uint32_t*)parmPage, 2 * NUM_PARAMS);

   if (crc == parmPage->crc)
   {
      DEBUG_SERIAL.println("parm_load: CRC match, loading parameters...");
      int loaded = 0;
      for (unsigned int idxPage = 0; idxPage < NUM_PARAMS; idxPage++)
      {
         Param::PARAM_NUM idx = Param::NumFromId(parmPage->data[idxPage].key);
         if (idx != Param::PARAM_INVALID && Param::GetType((Param::PARAM_NUM)idx) == Param::TYPE_PARAM)
         {
            const Param::Attributes *pAtr = Param::GetAttrib(idx);
            uint32_t fixedValue = parmPage->data[idxPage].value;
            float floatValue = FP_TOFLOAT(fixedValue);
            Param::SetFixed(idx, fixedValue);
            Param::SetFlagsRaw(idx, parmPage->data[idxPage].flags);
            DEBUG_SERIAL.printf("  [%d] %s (id=%d): 0x%08X fixed -> %.2f\r\n", 
                                idxPage, pAtr->name, pAtr->id, fixedValue, floatValue);
            loaded++;
         }
      }
      DEBUG_SERIAL.printf("parm_load: Successfully loaded %d parameters\r\n", loaded);
      return 0;
   }

   DEBUG_SERIAL.println("parm_load: CRC mismatch! Parameters NOT loaded");
   // Check if flash is erased (all 0xFF)
   if (parmPage->crc == 0xFFFFFFFF)
   {
      DEBUG_SERIAL.println("parm_load: Flash appears to be erased (no saved parameters)");
   }
   return -1;
}
