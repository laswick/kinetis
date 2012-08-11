#include "globalDefs.h"
#include "kinetis.h"
#include "hardware.h"

static flashConfig_t flashCfg;

int main(void)
{
    uint32_t buffer[3] = { 0x5a5a5a5a, 0x6a6a6a6a, 0x12345678 };

    flashInit(&flashCfg);
    flashErase(0x20000, FTFL_FLASH_SECTOR_SIZE);
    flashWrite(0x20000, buffer, 3);

    while (1)
        ;

    return 0;
}
