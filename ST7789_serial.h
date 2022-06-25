// ST7735, ILI9163, ST7789, ILI9488 use bidirectional SDA pin
// linking MOSI, MISO with a 470R is too big a load on SD card
// and you still have a noticeable phase shift from the wiring capacitance.
//
// Only STM32 can do bidirectional.   AVR, SAM, SAMD, ESP can not.  
// SAM, SAMD, K20, ESP, F072, L467, can do 9 bit.   AVR, F103, F401, F446 can not.
//
// L476 can do BIDIMODE with a good CS.  But it still clocks an "extra" byte.
// ST7735 requires 1 dummy clock cycle when reading GRAM.
// so we might just as well bit-bash on smaller controllers.
//
// I will try full L476 hardware on ILI9481 since it is 9-bit with CS.
// otherwise I will forget about bidirectional read on ILI9481.
//
// ILI9481 is painful for AVR, F103 to write
// ILI9481 is painful for everything to read (except F072, L476)
//
// Everything else with 8-bit SPI is fine.
// F072, L476, K20 4-16 bit SPI. SAM3X 8-16 bit.
//
// It looks difficult to mod the ST7789 board for CS.
// We can read hardware registers if we do EXACTLY the correct SCK clocks.
// Since readGRAM() is unlimited,  we must use CS to stop a read sequence

#include <SPI.h>

#if 0
#elif (defined(__AVR__) || defined(CORE_TEENSY)) && !defined(__IMXRT1062__) 
#warning RWREG_8_t
typedef uint8_t RWREG_t;
#elif defined(__arm__) || defined(ESP8266) || defined(ESP32)
typedef uint32_t RWREG_t;
#else
#error unsupported target
#endif

//8MHz is max for Saleae. 12MHz is max for ILI9481.  MODE0 reads better
static SPISettings settings(8000000, MSBFIRST, USE_MODE); 

static volatile RWREG_t *spicsPort, *spicdPort, *spimosiPort, *spiclkPort, *spirstPort;
static RWREG_t  spicsPinSet, spicdPinSet, spimosiPinSet, spiclkPinSet, spirstPinSet;

#if 0
#define PIN_LOW(p, b )       *spi ## p ## Port &= ~spi ## p ## PinSet
#define PIN_HIGH(p, b)       *spi ## p ## Port |= spi ## p ## PinSet
#define PIN_READ(p, b)       (*spi ## p ## Port & spi ## p ## PinSet)
#else
#define PIN_LOW(p, b)        digitalWrite(b, LOW)
#define PIN_HIGH(p, b)       digitalWrite(b, HIGH)
#define PIN_READ(p, b)       digitalRead(b)
#endif
#define PIN_OUTPUT(p, b)     pinMode(b, OUTPUT)
#define PIN_INPUT(p, b)      pinMode(b, INPUT)

#if defined(ESP32)
#define RESET_PIN 12
#define CD_PIN    13
#define CS_PIN    5
#define NO_CS_PIN 14
#define MOSI_PIN  23
#define SCK_PIN   18
#elif defined(ESP8266)
#define RESET_PIN D8
#define CD_PIN    D9
#define CS_PIN    D10
#define NO_CS_PIN D7
#define MOSI_PIN  D11
#define SCK_PIN   D13
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#define RESET_PIN 18
#define CD_PIN    19
#define CS_PIN    21
#define NO_CS_PIN 13
#define MOSI_PIN  3
#define SCK_PIN   2
#else
#define RESET_PIN 8
#define CD_PIN    9
#define CS_PIN    10
#define NO_CS_PIN 7
#define MOSI_PIN  11
#define SCK_PIN   13
#endif

#ifdef USE_NO_CS
#warning USE_NO_CS
#define CS_PIN    NO_CS_PIN
#define CS_PORT   NO_CS_PORT
#endif

#define CD_COMMAND PIN_LOW(cd, CD_PIN)
#define CD_DATA    PIN_HIGH(cd, CD_PIN)
#define CD_OUTPUT  PIN_OUTPUT(cd, CD_PIN)
#define CS_ACTIVE  PIN_LOW(cs, CS_PIN);
#define CS_IDLE    PIN_HIGH(cs, CS_PIN);
#define CS_OUTPUT  PIN_OUTPUT(cs, CS_PIN)
#define RESET_ACTIVE  PIN_LOW(rst, RESET_PIN)
#define RESET_IDLE    PIN_HIGH(rst, RESET_PIN)
#define RESET_OUTPUT  PIN_OUTPUT(rst, RESET_PIN)
#define SD_ACTIVE  PIN_LOW(sd, SD_PIN)
#define SD_IDLE    PIN_HIGH(sd, SD_PIN)
#define SD_OUTPUT  PIN_OUTPUT(sd, SD_PIN)
 // bit-bang macros for SDIO
#define SCK_LO     PIN_LOW(clk, SCK_PIN)
#define SCK_HI     PIN_HIGH(clk, SCK_PIN)
#define SCK_OUTPUT PIN_OUTPUT(clk, SCK_PIN)
#define MOSI_LO    PIN_LOW(mosi, MOSI_PIN)
#define MOSI_HI    PIN_HIGH(mosi, MOSI_PIN)
#define MOSI_OUTPUT PIN_OUTPUT(mosi, MOSI_PIN)
#define MOSI_IN    PIN_INPUT(mosi, MOSI_PIN)
#define LED_LO     PIN_LOW(led, LED_PIN)
#define LED_HI     PIN_HIGH(led, LED_PIN)
#define LED_OUT    PIN_OUTPUT(led, LED_PIN)
#define MOSI_READ  PIN_READ(mosi, MOSI_PIN)

#define FLUSH_IDLE { FLUSH(); CS_IDLE; }

#if defined(ARDUINO_ARDUINO_NANO33BLE)
#define USE_NANO33BLE 1
#endif
#if 0
#elif defined(ARDUINO_RASPBERRY_PI_PICO)
#define WRITE8(x)   { while( !spi_is_writable(spi0)) ; spi_get_hw(spi0)->dr = x; }
#define XCHG8(x,c)  { WRITE8(x);while( !(spi_is_readable(spi0))) ; c = spi_get_hw(spi0)->dr; }
#define FLUSH() {}  { while(spi_get_hw(spi0)->sr & SPI_SSPSR_BSY_BITS) ; while(spi_is_readable(spi0)) spi_get_hw(spi0)->dr; }
#elif USE_NANO33BLE == 1
#warning ARDUINO_ARDUINO_NANO33BLE WRITE8
#define NOP1 asm("nop")
#define NOP2 { NOP1; NOP1; }
#define NOP4 { NOP2; NOP2; }
#define NOP8 { NOP4; NOP4; }
#define NOP16 { NOP8; NOP8; }
#define NOP32 { NOP16; NOP16; }
#define NOP64 { NOP32; NOP32; }
#define NOPXX { NOP32; NOP8; NOP8; }  // delay works with 8MHz. NOP48 might be safer.
#define SPIX        NRF_SPI2    //chip is 8MHz max.  add delays before checking READY.
#define WRITE8(x)   { SPIX->TXD = x; NOPXX; while( !(SPIX->EVENTS_READY)) ; SPIX->RXD; }
#define XCHG8(x,c)  { if (SPIX->FREQUENCY != 0x8000000) SPIX->FREQUENCY = 0x80000000; SPIX->TXD = x; NOPXX; while( !(SPIX->EVENTS_READY)) ; c = SPIX->RXD; CS_ACTIVE; }
//#define XCHG8(x,c)  { c = SPI.transfer(x); }
#define FLUSH()     { }
#elif defined(__AVR_ATmega328P__)
#define WRITE8(x)   { SPDR = x; while ((SPSR & 0x80) == 0) ; }
#define XCHG8(x,c)  { SPDR = x; while ((SPSR & 0x80) == 0) ; c = SPDR; }
#define FLUSH()     { while (SPSR & 0x80) SPDR; }
#elif defined(ARDUINO_AVR_NANO_EVERY)
#define WRITE8(x)   { SPI0_DATA = x; while ((SPI0_INTFLAGS & 0x80) == 0) ; }
#define XCHG8(x,c)  { SPI0_DATA = x; while ((SPI0_INTFLAGS & 0x80) == 0) ; c = SPI0_DATA; }
#define FLUSH()     { while (SPI0_INTFLAGS & 0x80) SPI0_DATA; }
#elif defined(_ARDUINO_ARCH_STM32)
#warning ARDUINO_ARCH_STM32 WRITE8
#define WRITE8(x)   { while( !(SPI1->SR & SPI_SR_TXE)) ; *(uint8_t *)&SPI1->DR = x; }
#define XCHG8(x,c)  { WRITE8(x);while( !(SPI1->SR & SPI_SR_RXNE)) ; c = SPI1->DR; }
#define FLUSH()     { while(SPI1->SR & 0x80) ; while(SPI1->SR & SPI_SR_RXNE) SPI1->DR; }
#elif defined(ARDUINO_ARCH_SAMD)
#warning ARDUINO_ARCH_SAMD
#define WRITE8(x)   { while(SERCOM4->SPI.INTFLAG.bit.DRE == 0) ; SERCOM4->SPI.DATA.bit.DATA = x; }
#define XCHG8(x,c)  { WRITE8(x);while(SERCOM4->SPI.INTFLAG.bit.RXC == 0) ; c = SERCOM4->SPI.DATA.bit.DATA; }
#define FLUSH()     { while(SERCOM4->SPI.INTFLAG.bit.TXC == 0) ; while(SERCOM4->SPI.INTFLAG.bit.RXC) SERCOM4->SPI.DATA.bit.DATA; }
#elif defined(ARDUINO_SAM_DUE)  //write is ok.  read fails
#warning ARDUINO_SAM_DUE
#define WRITE8(x)   { while((SPI0->SPI_SR & SPI_SR_TDRE) == 0) ; SPI0->SPI_TDR = x | SPI_PCS(3); }
#define XCHG8(x,c)  { WRITE8(x);while((SPI0->SPI_SR & SPI_SR_RDRF) == 0) ; c = SPI0->SPI_RDR; }
#define FLUSH()     { while((SPI0->SPI_SR & SPI_SR_TXEMPTY) == 0) ; while(SPI0->SPI_SR & SPI_SR_RDRF) SPI0->SPI_RDR; }
#else
#define WRITE8(x)   { SPI.transfer(x); }
#define XCHG8(x,c)  { c = SPI.transfer(x); }
#define FLUSH()
#endif

#define write8 WRITE8

#if 0
#elif defined(ARDUINO_AVR_NANO_EVERY)
#define SDIO_INMODE()  {SPI.endTransaction(); MOSI_IN; SCK_OUTPUT;}
#elif defined(AVR)
#define SDIO_INMODE()  {SPCR = 0; MOSI_IN; SCK_OUTPUT;}
#else
#define SDIO_INMODE()  {SPI.endTransaction(); MOSI_IN;SCK_OUTPUT;}
#endif
#define SDIO_OUTMODE() {MOSI_OUTPUT;SCK_OUTPUT;SPI.beginTransaction(settings);}
//#define SDIO_OUTMODE() {SPI.beginTransaction(settings);}
static uint32_t readbits(uint8_t bits)
{
    uint32_t ret = 0;
    while (bits--) { //L476=4MHz, 328P=400kHz
        ret <<= 1;
#if USE_MODE == SPI_MODE3
        SCK_LO;   //MODE3
#endif
        SCK_HI;
        if (MOSI_READ) ret++;
#if USE_MODE == SPI_MODE0
        SCK_LO;   //MODE0
#endif
    }
    return ret;
}

static inline uint8_t xchg8(uint8_t x)
{
    uint8_t c;
    XCHG8(x, c);
    return c;
}
static inline void write16_N(uint16_t x, int16_t n)
{
#if defined(ESP8266) || defined(ESP32)
    uint16_t color = x;
    uint8_t hilo[2];
    hilo[0] = color >> 8;
    hilo[1] = color;
    SPI.writePattern(hilo, 2, (uint32_t)n);
#elif USE_NANO33BLE == 2
    uint8_t h = x >> 8, l = x;
    uint8_t buf[32], trashed[32];
    for (int i = 0; i < 32 && i < n * 2; ) { buf[i++] = h; buf[i++] = l; }
    while (n) {
        int cnt = n;
        if (cnt > 16) cnt = 16;
        memcpy(trashed, buf, cnt * 2); //dummy[] gets trashed by stupid Arduino method.
        SPI.transfer(trashed, cnt * 2);
        n -= cnt;
    }
#else
    uint8_t h = x >> 8, l = x;
    WRITE8(h);
    while (--n) {
        WRITE8(l);
        WRITE8(h);
    }
    WRITE8(l);
    FLUSH();
#endif
}
static inline void write24_N(uint16_t color, int16_t n)
{
#if defined(ESP8266) || defined(ESP32)
    uint8_t rgb[3];
    rgb[0] = color >> 8;
    rgb[1] = color >> 3;
    rgb[2] = color << 3;
    SPI.writePattern(rgb, 3, (uint32_t)n);
#elif USE_NANO33BLE == 2
    uint8_t r = color >> 8, g = (color >> 3), b = color << 3;
    uint8_t buf[48], trashed[48];
    for (int i = 0; i < 48 && i < n * 3; ) { buf[i++] = r; buf[i++] = g; buf[i++] = b; }
    while (n) {
        int cnt = n;
        if (cnt > 16) cnt = 16;
        memcpy(trashed, buf, cnt * 3); //dummy[] gets trashed by stupid Arduino method.
        SPI.transfer(trashed, cnt * 3);
        n -= cnt;
    }
#else
    uint8_t r = color >> 8, g = (color >> 3), b = color << 3, dummy;
    WRITE8(r);
    while (--n) {
        WRITE8(g);
        WRITE8(b);
        WRITE8(r);
    }
    WRITE8(g);
    WRITE8(b);
    FLUSH();
#endif
}
#define WriteCmd(cmd) { CD_COMMAND; CS_ACTIVE; xchg8(cmd); CD_DATA; }
#define WriteDat(dat) { write8(dat); }
#define write16(x) { write16_N(x, 1); }
static inline void write8_block(uint8_t *buf, int n)
{
#if defined(ESP8266) || defined(ESP32)
    SPI.writeBytes(buf, n);
#elif defined(ARDUINO_ARCH_STM32)
    uint8_t dummy[n];
    SPI.transfer(buf, dummy, n); 
#elif USE_NANO33BLE == 2
    uint8_t dummy[n];
    memcpy(dummy, buf, n);
    SPI.transfer(dummy, n); 
#else
    while (n--) WriteDat(*buf++);
#endif
}
static void INIT(void)
{
#if !defined(ARDUINO_ARCH_MBED)    
    spicsPort = portOutputRegister(digitalPinToPort(CS_PIN));
    spicsPinSet = digitalPinToBitMask(CS_PIN);
    spicdPort = portOutputRegister(digitalPinToPort(CD_PIN));
    spicdPinSet = digitalPinToBitMask(CD_PIN);
    spimosiPort = portInputRegister(digitalPinToPort(MOSI_PIN));
    spimosiPinSet = digitalPinToBitMask(MOSI_PIN);
    spiclkPort = portOutputRegister(digitalPinToPort(SCK_PIN));
    spiclkPinSet = digitalPinToBitMask(SCK_PIN);
    spirstPort = portOutputRegister(digitalPinToPort(RESET_PIN));
    spirstPinSet = digitalPinToBitMask(RESET_PIN);
#endif
    CS_IDLE;
    RESET_IDLE;
    CS_OUTPUT;
    RESET_OUTPUT;
    CD_OUTPUT;
    CD_DATA;
    MOSI_OUTPUT;
    SCK_OUTPUT;
#if USE_MODE == SPI_MODE3
    SCK_HI;
#elif USE_MODE == SPI_MODE0
    SCK_LO;                //probably reset default anyway
#endif
    SPI.endTransaction();  //ESP32 seems to require this
    SPI.begin();
    SPI.beginTransaction(settings);
    SPI.transfer(0);
}
