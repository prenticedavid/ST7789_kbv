#include <SPI.h>

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
#elif defined(ARDUINO_BLUEPILL_F103C8) || defined(ARDUINO_GENERIC_STM32F103C) //MY_BLUEPILL
#define RESET_PIN PA9
#define CD_PIN    PA10
#define CS_PIN    PB12
#define NO_CS_PIN PA3
#define MOSI_PIN  PB15
#define SCK_PIN   PB13
#elif defined(__AVR_ATmega328P__)
#define RESET_PIN PB0
#define CD_PIN    PB1
#define CS_PIN    PB2
#define NO_CS_PIN PD7
#define MOSI_PIN  PB3
#define SCK_PIN   PB5
#define RESET_PORT PORTB
#define CD_PORT PORTB
#define CS_PORT PORTB
#define NO_CS_PORT PORTD
#define SPI_PORT PORTB
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

#if defined(__AVR_ATmega328P__)
#define PIN_LOW(p, b)        (p) &= ~(1<<(b))
#define PIN_HIGH(p, b)       (p) |= (1<<(b))
#define PIN_OUTPUT(p, b)     *(&p-1) |= (1<<(b))
#else
#define PIN_LOW(p, b)        digitalWrite(b, LOW)
#define PIN_HIGH(p, b)       digitalWrite(b, HIGH)
#define PIN_OUTPUT(p, b)     pinMode(b, OUTPUT)
#define PIN_INPUT(p, b)      pinMode(b, INPUT_PULLUP)
#define PIN_READ(p, b)       digitalRead(b)
#endif

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
#define SCK_OUTPUT    PIN_OUTPUT(SPI_PORT, SCK_PIN)
#define MOSI_LO    PIN_LOW(SPI_PORT, MOSI_PIN)
#define MOSI_HI    PIN_HIGH(SPI_PORT, MOSI_PIN)
#define MOSI_OUTPUT   PIN_OUTPUT(SPI_PORT, MOSI_PIN)
#define MOSI_IN    PIN_INPUT(SPI_PORT, MOSI_PIN)
#define LED_LO     PIN_LOW(LED_PORT, LED_PIN)
#define LED_HI     PIN_HIGH(LED_PORT, LED_PIN)
#define LED_OUT    PIN_OUTPUT(LED_PORT, LED_PIN)

/*
#define CD_COMMAND digitalWrite(CD_PIN, LOW)
#define CD_DATA    digitalWrite(CD_PIN, HIGH)
#define CD_OUTPUT  pinMode(CD_PIN, OUTPUT)
#define CS_ACTIVE  digitalWrite(CS_PIN, LOW)
#define CS_IDLE    digitalWrite(CS_PIN, HIGH)
#define CS_OUTPUT  pinMode(CS_PIN, OUTPUT)
#define RESET_ACTIVE  digitalWrite(RESET_PIN, LOW)
#define RESET_IDLE    digitalWrite(RESET_PIN, HIGH)
#define RESET_OUTPUT  pinMode(RESET_PIN, OUTPUT)
#define MOSI_LO    digitalWrite(MOSI_PIN, LOW)
#define MOSI_HI    digitalWrite(MOSI_PIN, HIGH)
#define MOSI_OUTPUT pinMode(MOSI_PIN, OUTPUT)
#define SCK_LO     digitalWrite(SCK_PIN, LOW)
#define SCK_HI     digitalWrite(SCK_PIN, HIGH)
#define SCK_OUTPUT pinMode(SCK_PIN, OUTPUT)
*/

#define FLUSH_IDLE CS_IDLE

static SPISettings settings(12000000, MSBFIRST, SPI_MODE3); //8MHz is max for Saleae. 12MHz is max for ILI9481

#if 1

#if defined(__AVR_ATmega328P__)
#define WRITE8(x)   { SPDR = x; }
#define READ8(c)    { while ((SPSR & 0x80) == 0) ; c = SPDR; }
#define FLUSH()
#elif defined(ARDUINO_ARCH_STM32)
#warning ARDUINO_ARCH_STM32
#define WRITE8(x)   { while( !(SPI1->SR & SPI_SR_TXE)) ; *(uint8_t *)&SPI1->DR = x; }
#define READ8(c)    { }
#define FLUSH()     { while(SPI1->SR & 0x80) ; }
#else
#define WRITE8(x)   { SPI.transfer(x); }
#define READ8(c)    { }
#define FLUSH()
#endif

#define write8 xchg8
static inline uint8_t xchg8(uint8_t c)
{
    WRITE8(c);
    READ8(c);
    FLUSH();
    return c;
}
static inline void write16_N(uint16_t x, int16_t n)
{
    uint8_t h = x >> 8, l = x, dummy;
    WRITE8(h);
    while (--n) {
        READ8(dummy);
        WRITE8(l);
        READ8(dummy);
        WRITE8(h);
    }
    READ8(dummy);
    WRITE8(l);
    READ8(dummy);
    FLUSH();
}
static inline void write24_N(uint16_t color, int16_t n)
{
	uint8_t r = color >> 8, g = (color >> 3), b = color << 3, dummy;
    WRITE8(r);
    while (--n) {
        READ8(dummy);
        WRITE8(g);
        READ8(dummy);
        WRITE8(b);
        READ8(dummy);
        WRITE8(r);
    }
    READ8(dummy);
    WRITE8(g);
    READ8(dummy);
    WRITE8(b);
    READ8(dummy);
    FLUSH();
}
#define WriteCmd(cmd) { CD_COMMAND; CS_ACTIVE; write8(cmd); CD_DATA; }
#define WriteDat(dat) { write8(dat); }
#define write16(x) { write16_N(x, 1); }
static inline void write8_block(uint8_t *buf, int n)
{
    while (n--) WriteDat(*buf++);
}
static void INIT(void)
{
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
#else
static inline void write9(uint8_t c, uint8_t dc)
{
#if defined(ESP32) && USE_SPI && USE_9BIT
    uint32_t out, d = c;
    if (dc) d |= 0x100;
    d <<= 7;    //shift to nearest byte boundary. remove mask from HAL function.
    //alternatively use regular user values.  let HAL shift to byte boundary.
    SPI.transferBits(d, NULL, 9);
    return;
#endif
#if USE_9BIT
    if (dc) MOSI_HI;
    else MOSI_LO;
    SCK_LO;
    SCK_HI;
#else
    if (dc) CD_DATA;
    else CD_COMMAND;
#endif
#if USE_SPI
    SPI.transfer(c);
#else
    for (int i = 0; i < 8; i++) {
        if (c & 0x80) MOSI_HI;
        else MOSI_LO;
        SCK_LO;
        SCK_HI;
        c <<= 1;
    }
#endif
#if !USE_9BIT
    CD_DATA; //.kbv
#endif
}

static inline void write8_block(uint8_t *buf, int n)
{
#if USE_SPI && !USE_9BIT
    CD_DATA;
#if defined(__STM32F1__)  //weird maple
    SPI.write(buf, n);
#else
    SPI.transfer(buf, n);
#endif
#else
    while (n--) write9(*buf++, 1);
#endif
}

static inline void WriteCmd(uint8_t c)
{
    CS_ACTIVE;
    write9(c, 0);
}

static inline void WriteDat(uint8_t c)
{
    write9(c, 1);
}

static void INIT(void)
{
    CS_IDLE;
    RESET_IDLE;
    CS_OUTPUT;
    RESET_OUTPUT;
    CD_OUTPUT;
    CD_DATA;
    MOSI_OUTPUT;
    SCK_OUTPUT;
    SCK_HI;
#if USE_SPI
    SPI.begin();
    SPI.beginTransaction(settings);
#endif
}
#endif
