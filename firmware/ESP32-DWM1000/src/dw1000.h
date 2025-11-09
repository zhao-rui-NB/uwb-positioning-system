


#ifndef __DW1000_H__
#define __DW1000_H__


#include "deca_types.h"
#include "deca_device_api.h"

#define PIN_DW1000_RST 16
#define PIN_DW1000_IRQ 4
#define PIN_DW1000_SS 17



#ifdef __cplusplus
extern "C" {
#endif

void init_DW1000();
void reset_DW1000();
void set_dw1000_spi_rate_fast(bool fast);
int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodyLength, const uint8 *bodyBuffer);
int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer);


void lcd_display_str(const char *str);
void deca_sleep(unsigned int time_ms);
void Sleep(int time_ms);
void port_set_dw1000_slowrate();
void port_set_dw1000_fastrate();

decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t s);


#ifdef __cplusplus
}
#endif


#endif