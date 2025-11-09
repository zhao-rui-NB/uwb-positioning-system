
#include <Arduino.h>
#include <SPI.h>

#include "dw1000.h"
#include "deca_device_api.h"
#include "uwb.h"

SPISettings dw1000_spi_slow_setting = SPISettings(2000000, MSBFIRST, SPI_MODE0); // 2MHz
SPISettings dw1000_spi_fast_setting = SPISettings(2000000, MSBFIRST, SPI_MODE0); // 8MHz
SPISettings * dw1000_spi_current_setting = &dw1000_spi_slow_setting;


void init_DW1000(){
    pinMode(PIN_DW1000_RST, OUTPUT);
    
    pinMode(PIN_DW1000_IRQ, INPUT);

    pinMode(PIN_DW1000_SS, OUTPUT);
    digitalWrite(PIN_DW1000_SS, HIGH); // SS high
    
    SPI.begin();

    reset_DW1000();


}


void reset_DW1000() {
    digitalWrite(PIN_DW1000_RST, LOW);
    delay(10);
    digitalWrite(PIN_DW1000_RST, HIGH);
    delay(10);
}

void set_dw1000_spi_rate_fast(bool fast) {
    if(fast)
        dw1000_spi_current_setting = &dw1000_spi_fast_setting;
    else
        dw1000_spi_current_setting = &dw1000_spi_slow_setting;
}

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodyLength, const uint8 *bodyBuffer) {
    // decaIrqStatus_t  stat ;
    // stat = decamutexon() ;

    digitalWrite(PIN_DW1000_SS, LOW); // Make sure the SS pin is low

    // header
    SPI.beginTransaction(*dw1000_spi_current_setting);
    SPI.transferBytes(headerBuffer, NULL, headerLength);
    // data
    SPI.transferBytes(bodyBuffer, NULL, bodyLength);
    SPI.endTransaction();

    digitalWrite(PIN_DW1000_SS, HIGH); // Make sure the SS pin is high

    // decamutexoff(stat);

    return 0;
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
    int i;
    // decaIrqStatus_t  stat ;
    // stat = decamutexon() ;

    digitalWrite(PIN_DW1000_SS, LOW); // Make sure the SS pin is low

    // header
    SPI.beginTransaction(*dw1000_spi_current_setting);
    SPI.transferBytes(headerBuffer, NULL, headerLength);
    // data
    SPI.transferBytes(NULL, readBuffer, readlength);
    SPI.endTransaction();

    digitalWrite(PIN_DW1000_SS, HIGH); // Make sure the SS pin is high

    // decamutexoff(stat);

    return 0;
} // end readfromspi()

void Sleep(int time_ms){delay(time_ms);}
void deca_sleep(unsigned int time_ms){delay(time_ms);}
void lcd_display_str(const char *str){Serial.printf("[LCD PRINT] %s\n", str);}

void port_set_dw1000_slowrate(){set_dw1000_spi_rate_fast(false);}
void port_set_dw1000_fastrate(){set_dw1000_spi_rate_fast(true);}


bool is_irq_attached = false;

decaIrqStatus_t decamutexon(void) {
	// check if interrupt is attached, if so, disable it and return true
    decaIrqStatus_t s = is_irq_attached;
	if(s) {
        detachInterrupt(PIN_DW1000_IRQ); // disable interrupt
	}
	return s ;   // return state before disable, value is used to re-enable in decamutexoff call
}

/// set the interrupt state, enable interrupt if s is true
void decamutexoff(decaIrqStatus_t s) {        
	if(s) {
        // attachInterrupt(PIN_DW1000_IRQ, dwt_isr, RISING); // enable interrupt, all
        attachInterrupt(PIN_DW1000_IRQ, uwb_irq_handler, RISING); // 
    }
}


// ARDUINO_ISR_ATTR 