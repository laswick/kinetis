#ifndef __XMODEM_H__
#define __XMODEM_H__
#include "hardware.h"

typedef struct {
    uartIF_t *uartPtr;
    int32_t   numRetries;
} xmodemCfg_t;

extern int32_t xmodemInit(xmodemCfg_t *cfg);
extern int32_t xmodemAbort(void);
extern int32_t xmodemRecv(uint8_t *outBuffer, uint32_t numBytes);
#endif
