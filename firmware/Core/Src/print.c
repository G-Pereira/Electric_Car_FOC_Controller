#include "stm32f1xx.h"

uint32_t my_ITM_SendChar(uint32_t ch, uint32_t port)
{
  if (((ITM->TCR & ITM_TCR_ITMENA_Msk) != 0UL) &&      /* ITM enabled */
      ((ITM->TER & 1UL               ) != 0UL)   )     /* ITM Port #0 enabled */
  {
    while (ITM->PORT[port].u32 == 0UL)
    {
      __NOP();
    }
    ITM->PORT[port].u8 = (uint8_t)ch;
  }
  return (ch);
}

int myprintf(const char *ptr, uint32_t port)
{
 int i=0;
 for(i=0 ; (*ptr++) != '\0' ; i++)
  my_ITM_SendChar((*ptr++), port);
 return i;
}
