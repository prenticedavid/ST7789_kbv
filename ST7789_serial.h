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
#elif defined(__AVR__) || defined(CORE_TEENSY)
typedef uint8_t RWREG_t;
#elif defined(__arm__) || defined(ESP8266) || defined(ESP32)
typedef uint32_t RWREG_t;
#else
#error unsupported target
#endif

static volatile RWREG_t *spicsPort, *spicdPort, *spimosiPort, *spiclkPort, *spirstPort;
static RWREG_t  spicsPinSet, spicdPinSet, spimosiPinSet, spiclkPinSet, spirstPinSet;

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

#define CD_COMMAND *spicdPort &= ~spicdPinSet
#define CD_DATA    *spicdPort |= spicdPinSet
#define CD_OUTPUT  pinMode(CD_PIN, OUTPUT)
#define CS_ACTIVE  *spicsPort &= ~spicsPinSet
#define CS_IDLE    *spicsPort |= spicsPinSet
#define CS_OUTPUT  pinMode(CS_PIN, OUTPUT)
#define RESET_ACTIVE  *spirstPort &= ~spirstPinSet
#define RESET_IDLE    *spirstPort |= spirstPinSet
#define RESET_OUTPUT  pinMode(RESET_PIN, OUTPUT)
 // bit-bang macros for SDIO
#define SCK_LO     *spiclkPort &= ~spiclkPinSet
#define SCK_HI     *spiclkPort |= spiclkPinSet
#define SCK_OUTPUT    pinMode(SCK_PIN, OUTPUT)
#define MOSI_LO     *spimosiPort &= ~spimosiPinSet
#define MOSI_HI     *spimosiPort |= spimosiPinSet
#define MOSI_OUTPUT pinMode(MOSI_PIN, OUTPUT)
#define MOSI_IN  pinMode(MOSI_PIN, INPUT)  //AVR is happier but still glitches
#define MOSI_READ   (*spimosiPort & spimosiPinSet)

#define FLUSH_IDLE { FLUSH(); CS_IDLE; }

//8MHz is max for Saleae. 12MHz is max for ILI9481.  MODE0 reads better
static SPISettings settings(8000000, MSBFIRST, SPI_MODE3); 

#if defined(__AVR_ATmega328P__)
#define WRITE8(x)   { SPDR = x; while ((SPSR & 0x80) == 0) ; }
#define READ8(c)    { while ((SPSR & 0x80) == 0) ; c = SPDR; }
#define FLUSH()     { while (SPSR & 0x80) SPDR; }
#elif defined(ARDUINO_ARCH_STM32)
#warning ARDUINO_ARCH_STM32 WRITE8
#define WRITE8(x)   { while( !(SPI1->SR & SPI_SR_TXE)) ; *(uint8_t *)&SPI1->DR = x; }
#define XCHG8(x,c)  { WRITE8(x);while( !(SPI1->SR & SPI_SR_RXNE)) ; c = SPI1->DR; }
#define FLUSH()     { while(SPI1->SR & 0x80) ; while(SPI1->SR & SPI_SR_RXNE) SPI1->DR; }
#elif defined(ARDUINO_ARCH_SAMD)
#warning ARDUINO_ARCH_SAMD
#define WRITE8(x)   { while(SERCOM4->SPI.INTFLAG.bit.DRE == 0) ; SERCOM4->SPI.DATA.bit.DATA = x; }
#define XCHG8(x,c)  { WRITE8(x);while(SERCOM4->SPI.INTFLAG.bit.RXC == 0) ; c = SERCOM4->SPI.DATA.bit.DATA; }
#define FLUSH()     { while(SERCOM4->SPI.INTFLAG.bit.TXC == 0) ; while(SERCOM4->SPI.INTFLAG.bit.RXC) SERCOM4->SPI.DATA.bit.DATA; }
#else
#define WRITE8(x)   { SPI.transfer(x); }
#define XCHG8(x,c)  { c = SPI.transfer(x); }
#define FLUSH()
#endif

#define write8 WRITE8

#if defined(AVR)
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
        SCK_LO;   //MODE3
        SCK_HI;
        if (MOSI_READ) ret++;
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
    uint8_t color = x, hilo[2];
    hilo[0] = color >> 8;
    hilo[1] = color;
    SPI.writePattern(hilo, 2, (uint32_t)n);
#else
    uint8_t h = x >> 8, l = x, dummy;
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
    while (n--) WriteDat(*buf++);
}
static void INIT(void)
{
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
    CS_IDLE;
    RESET_IDLE;
    CS_OUTPUT;
    RESET_OUTPUT;
    CD_OUTPUT;
    CD_DATA;
    MOSI_OUTPUT;
    SCK_OUTPUT;
    SCK_HI;
    SPI.begin();
    SPI.beginTransaction(settings);
}
