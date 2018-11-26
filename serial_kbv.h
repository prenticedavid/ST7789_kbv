#if defined(__AVR_ATmega328P__) && defined(ILI9488_KBV_H_) 
//#define USE_SERIAL_COMPLEX     //optimised C code for Uno, Xmega, ...
#endif
#if defined(__STM32F1__)
#define DMA__STM32F1__         //special feature of MAPLE CORE
#endif
//#define MY_BLUEPILL
// MAPLE core has SPI.write(block, n)
// ST core has only got SPI.transfer(block, n)
// SAMD core might have SPI.transfer(block, n)
// SAM core has got SPI.transfer(block, n)
// so it is probably safest to have a moderate stack buffer and use transfer

//ST core is ok for ILI9341 but not for ST7735X

#define CD_COMMAND PIN_LOW(CD_PORT, CD_PIN)
#define CD_DATA    PIN_HIGH(CD_PORT, CD_PIN)
#define CD_OUTPUT  PIN_OUTPUT(CD_PORT, CD_PIN)
#define CS_ACTIVE  PIN_LOW(CS_PORT, CS_PIN);
#define CS_IDLE    PIN_HIGH(CS_PORT, CS_PIN);
#define CS_OUTPUT  PIN_OUTPUT(CS_PORT, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(RESET_PORT, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(RESET_PORT, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(RESET_PORT, RESET_PIN)
#define SD_ACTIVE  PIN_LOW(SD_PORT, SD_PIN)
#define SD_IDLE    PIN_HIGH(SD_PORT, SD_PIN)
#define SD_OUTPUT  PIN_OUTPUT(SD_PORT, SD_PIN)
 // bit-bang macros for SDIO
#define SCK_LO     PIN_LOW(SPI_PORT, SCK_PIN)
#define SCK_HI     PIN_HIGH(SPI_PORT, SCK_PIN)
#define SCK_OUT    PIN_OUTPUT(SPI_PORT, SCK_PIN)
#define MOSI_LO    PIN_LOW(SPI_PORT, MOSI_PIN)
#define MOSI_HI    PIN_HIGH(SPI_PORT, MOSI_PIN)
#define MOSI_OUT   PIN_OUTPUT(SPI_PORT, MOSI_PIN)
#define MOSI_IN    PIN_INPUT(SPI_PORT, MOSI_PIN)
#define LED_LO     PIN_LOW(LED_PORT, LED_PIN)
#define LED_HI     PIN_HIGH(LED_PORT, LED_PIN)
#define LED_OUT    PIN_OUTPUT(LED_PORT, LED_PIN)

#if defined(USE_SERIAL_COMPLEX)
#include "serial_complex.h"
#elif defined(__AVR_ATxmega32A4U__) || defined(__AVR_ATxmega128A4U__)
#include "serial_xmega.h"
#define xchg8(x)     xchg8_1(x)
#define WriteCmd(x)  { CD_COMMAND; xchg8_1(x); CD_DATA; }
#define wait_ms(ms)  delay(ms)
#define write16(x)   { write16_N(x, 1); }
#define write24(x)   { write24_N(x, 1); }
#define WriteData(x) { write16(x); }

#else

#include <SPI.h>                //include before write16() macro
#warning Using Arduino SPI methods

#if defined(NINEBITS)
#define xchg8(x)     readbits(8)
#define WriteCmd(x)  { SDIO_OUTMODE(); MOSI_LO; SCK_HI; SCK_LO; write_8(x); }
#define WriteDat8(x) { MOSI_HI; SCK_HI; SCK_LO; write_8(x); }
#define INIT()  { CS_IDLE; RESET_IDLE; SETDDR; }
#define SDIO_INMODE()  MOSI_IN;SCK_OUT    //no braces
#define SDIO_OUTMODE() {MOSI_OUT;SCK_OUT;}
#else
#define xchg8(x)     xchg8_1(x)
#define WriteCmd(x)  { CD_COMMAND; xchg8_1(x); CD_DATA; }
#define INIT()  { CS_IDLE; RESET_IDLE; SETDDR; SPI.begin(); SPI.beginTransaction(settings); }
#define SDIO_INMODE()  SPI.endTransaction(); MOSI_IN;SCK_OUT    //no braces
#define SDIO_OUTMODE() {MOSI_OUT;SCK_OUT;SPI.beginTransaction(settings);}
#endif

#define wait_ms(ms)  delay(ms)
#define write16(x)   { write16_N(x, 1); }
#define write24(x)   { write24_N(x, 1); }
#define WriteData(x) { write16(x); }

static uint8_t spibuf[16];

#if 0
#elif defined(ESP32)
#define CD_PIN     13
#define CS_PIN     5
#define RESET_PIN  12
#define SD_PIN     17
#define MOSI_PIN   23
#define SCK_PIN    18
#elif defined(ESP8266)
#define CD_PIN     D9
#define CS_PIN     D10
#define RESET_PIN  D8
#define SD_PIN     D4
#define MOSI_PIN   D11
#define SCK_PIN    D13
#elif defined(MY_BLUEPILL)
#define CD_PIN     PA10
#define CS_PIN     PB12
#define RESET_PIN  PA9
#define SD_PIN     PA0
#define MOSI_PIN   PB15
#define SCK_PIN    PB13
#else
#define CD_PIN     9
#define CS_PIN     7         //.kbv
#define RESET_PIN  8
#define SD_PIN     4
#define MOSI_PIN   11
#define SCK_PIN    13
#endif

#define SETDDR  { CS_OUTPUT; CD_OUTPUT; RESET_OUTPUT; PIN_HIGH(SD_PORT, SD_PIN); PIN_OUTPUT(SD_PORT, SD_PIN); }

#define PIN_LOW(p, b)        digitalWrite(b, LOW)
#define PIN_HIGH(p, b)       digitalWrite(b, HIGH)
#define PIN_OUTPUT(p, b)     pinMode(b, OUTPUT)
#define PIN_INPUT(p, b)      pinMode(b, INPUT_PULLUP)
#define PIN_READ(p, b)       digitalRead(b)

#define FLUSH_IDLE { CS_IDLE; }

static SPISettings settings(8000000, MSBFIRST, SPI_MODE3);

static inline void write_8(uint8_t val)
{
    for (uint8_t i = 0; i < 8; i++) {   //send command
        if (val & 0x80) MOSI_HI;
	    else MOSI_LO;
		SCK_HI;
		SCK_LO;
        val <<= 1;
    }
}

static inline uint8_t xchg8_1(uint8_t x)
{
#if defined(DMA__STM32F1__)
    uint8_t ret;
	SPI.dmaTransfer(&x, &ret, 1);
	return ret;
#else
	return SPI.transfer(x);
#endif
}

static uint32_t readbits(uint8_t bits)
{
	uint32_t ret = 0;
	while (bits--) {
		ret <<= 1;
		if (PIN_READ(SPI_PORT, MOSI_PIN))
		    ret++;
		SCK_HI;
		SCK_LO;
	}
	return ret;
}

static inline void write16_N(uint16_t color, int16_t n)
{
#if defined(NINEBITS)
	uint8_t hi = color >> 8, lo = color;
	while (n-- > 0) {
		WriteDat8(hi);
		WriteDat8(lo);
	}
#elif defined(ESP8266) || defined(ESP32)
    uint8_t hilo[2];
	hilo[0] = color >> 8;
	hilo[1] = color;
	SPI.writePattern(hilo, 2, (uint32_t)n);
#elif defined(DMA__STM32F1__)
  SPI.setDataSize (SPI_CR1_DFF); // Set SPI 16bit mode
    SPI.dmaSend(&color, n, 0);
  SPI.setDataSize (0);
#elif defined(__STM32F1__)
    uint8_t buf[64];
	int cnt = (n > 32) ? 32 : n;
	uint8_t *p = buf;
	while (cnt--) { *p++ = color >> 8; *p++ = color; }
	while (n > 0) {
		cnt = (n > 32) ? 32 : n;
		SPI.write(buf, cnt << 1);
		n -= cnt;
	}
#elif 1
    uint8_t buf[64];
	while (n > 0) {
    	uint8_t *p = buf;
    	int cnt = (n > 32) ? 32 : n;
    	while (cnt--) { *p++ = color >> 8; *p++ = color; }
		cnt = (n > 32) ? 32 : n;
		SPI.transfer(buf, cnt << 1);
		n -= cnt;
	}
#else
	uint8_t hi = color >> 8, lo = color;
	while (n-- > 0) {
		SPI.transfer(hi);
		SPI.transfer(lo);
	}
#endif
}

static inline void write24_N(uint16_t color, int16_t n)
{
#if defined(NINEBITS)
	uint8_t r = color >> 8, g = (color >> 3), b = color << 3;
	while (n-- > 0) {
		WriteDat8(r);
		WriteDat8(g);
		WriteDat8(b);
	}
#elif defined(ESP8266) || defined(ESP32)
    uint8_t rgb[3];
	rgb[0] = color >> 8;
	rgb[1] = color >> 3;
	rgb[2] = color << 3;
	SPI.writePattern(rgb, 3, (uint32_t)n);
#else
	uint8_t r = color >> 8, g = (color >> 3), b = color << 3;
	while (n-- > 0) {
		SPI.transfer(r);
		SPI.transfer(g);
		SPI.transfer(b);
	}
#endif
}

static inline void write8_block(uint8_t * block, int16_t n, uint8_t bigend = 0)
{
#if defined(NINEBITS)
    while (n-- > 0) WriteDat8(*block++);
#elif defined(ESP8266) || defined(ESP32)
	SPI.writeBytes(block, (uint32_t)n);
#elif defined(DMA__STM32F1__)
    SPI.dmaSend(block, n, 1);
#elif defined(__STM32F1__)
	SPI.write(block, (uint32_t)n);
#elif defined(__SAMD21G18A__)
    Sercom *s = SERCOM1; 
	if (bigend) {
        n >>= 1;
        while (n--) {
	        uint8_t l = *block++, h = *block++;
			while (s->SPI.INTFLAG.bit.DRE == 0) ;
            s->SPI.DATA.bit.DATA = h;
			while (s->SPI.INTFLAG.bit.DRE == 0) ;
            s->SPI.DATA.bit.DATA = l;
        }	
	}
    else {	
	while (n--) {
	    while (s->SPI.INTFLAG.bit.DRE == 0) ;
        s->SPI.DATA.bit.DATA = *block++;
	}
    }
	while (s->SPI.INTFLAG.bit.TXC == 0) ;
    s->SPI.DATA.bit.DATA;
    s->SPI.DATA.bit.DATA;
#else
	SPI.transfer(block, n);
#endif
}

#if defined(__SAMD21G18A__)
static inline void spi_pattern(uint8_t * txbuf, uint16_t * rxbuf, int16_t n, int16_t rpt = 0)
{
    Sercom *s = SERCOM1;
	uint8_t cmd, reply;
    do {	
	    for (int i = n; i--; ) {
	        cmd = (txbuf == NULL) ? 0 : *txbuf++; 
			while (s->SPI.INTFLAG.bit.DRE == 0) ;
            s->SPI.DATA.bit.DATA = cmd;
			if (s->SPI.INTFLAG.bit.RXC) {
				reply = s->SPI.DATA.bit.DATA;
			    if (rxbuf) *rxbuf++ = reply;
            }
        }
	} while (--rpt > 0);
    while (s->SPI.INTFLAG.bit.TXC == 0) ;
    reply = s->SPI.DATA.bit.DATA;
    if (rxbuf) *rxbuf++ = reply;
}
#endif

#endif
