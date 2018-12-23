//to do: check MBED timing
//to do: read ID at faster SPI speeds

#include "ST7789_kbv.h"
#include <SPI.h>

#define USE_9BIT  0

#define RESET_PIN 8
#define CD_PIN    9
#define CS_PIN    7
#define MOSI_PIN  11
#define SCK_PIN   13
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

#define FLUSH_IDLE CS_IDLE
static SPISettings settings(8000000, MSBFIRST, SPI_MODE3);

static inline void write9(uint8_t c, uint8_t dc)
{
#if USE_9BIT == 0
    if (dc) CD_DATA;
    else CD_COMMAND;
    SPI.transfer(c);
#else
    if (dc) MOSI_HI;
    else MOSI_LO;
    SCK_LO;
    SCK_HI;
    for (int i = 0; i < 8; i++) {
        if (c & 0x80) MOSI_HI;
        else MOSI_LO;
        SCK_LO;
        SCK_HI;
        c <<= 1;
    }
#endif
}

static void INIT(void)
{
    CS_IDLE;
    RESET_IDLE;
    CS_OUTPUT;
    RESET_OUTPUT;
    CD_OUTPUT;
    MOSI_OUTPUT;
    SCK_OUTPUT;
    SCK_HI;
#if USE_9BIT == 0
    SPI.begin();
    SPI.beginTransaction(settings);
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

static inline void write16(uint16_t x)
{
    WriteDat(x >> 8);
    WriteDat(x & 0xFF);
}

static inline void writeColor(uint16_t x, uint32_t N)
{
    // alter this function for 666 format controllers
    uint8_t h = (x >> 8), l = x;
    while (N--) {
        WriteDat(h);
        WriteDat(l);
    }
}

ST7789_kbv::ST7789_kbv(): Adafruit_GFX(240, 240)
{
}

static uint8_t done_reset;

void ST7789_kbv::reset(void)
{
    done_reset = 1;
    INIT();
    RESET_IDLE;
    delay(50);
    RESET_ACTIVE;
    delay(100);
    RESET_IDLE;
    delay(100);
}

void ST7789_kbv::pushCommand(uint16_t cmd, uint8_t * block, int8_t N)
{
    WriteCmd(cmd);
    while (N--) WriteDat(*block++);
    FLUSH_IDLE;
}

uint16_t ST7789_kbv::readID(void)                          //{ return 0x9341; }
{
    if (!done_reset) reset();
    return 0x7789;
}

uint16_t ST7789_kbv::readReg(uint16_t reg, uint8_t idx)
{
    return 0x1234;
}

int16_t ST7789_kbv::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
    return 1;
}

void ST7789_kbv::setRotation(uint8_t r)
{
    uint8_t mac = 0x00;
    Adafruit_GFX::setRotation(r & 3);
    switch (rotation) {
        case 0:
            mac = 0x08;
            break;
        case 1:        //LANDSCAPE 90 degrees
            mac = 0x68;
            break;
        case 2:
            mac = 0xD8;
            break;
        case 3:
            mac = 0xB8;
            break;
    }
    pushCommand(0x36, &mac, 1);
    uint8_t d[3] = {0x27, 0x00, 0x10}; //regular 240x320
    if (HEIGHT == 240 && rotation & 2) d[1] = 0x0A;     //fix GateScan on 240x240 panel
    pushCommand(0xE4, d, 3);
}

void ST7789_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // ILI934X just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
    CS_ACTIVE;
    WriteCmd(0x2A);
    write16(x);
    WriteCmd(0x2B);
    write16(y);
    WriteCmd(0x2C);
    writeColor(color, 1);
    FLUSH_IDLE;
}

void ST7789_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
    CS_ACTIVE;
    WriteCmd(0x2A);
    write16(x);
    write16(x1);
    WriteCmd(0x2B);
    write16(y);
    write16(y1);
    FLUSH_IDLE;
}

void ST7789_kbv::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
    int16_t end;
    if (w < 0) {
        w = -w;
        x -= w;
    }                           //+ve w
    end = x + w;
    if (x < 0)
        x = 0;
    if (end > width())
        end = width();
    w = end - x;
    if (h < 0) {
        h = -h;
        y -= h;
    }                           //+ve h
    end = y + h;
    if (y < 0)
        y = 0;
    if (end > height())
        end = height();
    h = end - y;
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    WriteCmd(0x2C);
    if (h > w) {
        end = h;
        h = w;
        w = end;
    }
    while (h-- > 0) {
        writeColor(color, w);
    }
    FLUSH_IDLE;
    setAddrWindow(0, 0, width() - 1, height() - 1);
}

void ST7789_kbv::pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, bool first, uint8_t flags)
{
    uint8_t h, l;
    bool isconst = flags & 1;
    bool isbigend = (flags & 2) != 0;
    if (first) {
        WriteCmd(cmd);
    }
    while (n-- > 0) {
        if (isconst) {
            h = pgm_read_byte(block++);
            l = pgm_read_byte(block++);
        } else {
            h = (*block++);
            l = (*block++);
        }
        uint16_t color;
        if (isbigend) color = h << 8 | l;
        else color = l << 8 | h;
        writeColor(color, 1);
    }
}

void ST7789_kbv::pushColors(uint16_t * block, int16_t n, bool first)
{
    pushColors_any(0x2C, (uint8_t *)block, n, first, 0);
}
void ST7789_kbv::pushColors(uint8_t * block, int16_t n, bool first)
{
    pushColors_any(0x2C, (uint8_t *)block, n, first, 2);   //regular bigend
}
void ST7789_kbv::pushColors(const uint8_t * block, int16_t n, bool first, bool bigend)
{
    pushColors_any(0x2C, (uint8_t *)block, n, first, bigend ? 3 : 1);
}

void ST7789_kbv::invertDisplay(boolean i)
{
    pushCommand(i ? 0x21 : 0x20, NULL, 0);
}

void ST7789_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    int16_t bfa = 320 - top - scrollines;  // bottom fixed area
    int16_t vsp;
    vsp = top + offset;  // vertical start position
    if (offset < 0)
        vsp += scrollines;          //keep in unsigned range
    WriteCmd( 0x33);
    write16(top);        //TOP
    write16(scrollines); //VSA
    write16(bfa);        //BFA

    WriteCmd(0x37);
    write16(vsp);        //VLSP
    FLUSH_IDLE;
}

#define TFTLCD_DELAY8 0xFF
static const uint8_t ST7789_regValues_Adafruit[] PROGMEM = {
    0x01, 0,            //Soft Reset
    TFTLCD_DELAY8, 150,  // .kbv will power up with ONLY reset, sleep out, display on
    0x28, 0,            //Display Off
    0x3A, 1, 0x55,      //Pixel read=565, write=565.
    //            (0xB2), 5, 0x0C, 0x0C, 0x00, 0x33, 0x33,    //PORCTRK: Porch setting [08 08 00 22 22] PSEN=0 anyway
    //            (0xB7), 1, 0x35,    //GCTRL: Gate Control [35]
    //            (0xBB), 1, 0x2B,    //VCOMS: VCOM setting VCOM=1.175 [20] VCOM=0.9
    (0xC0), 1, 0x1C,    //LCMCTRL: LCM Control [2C] was=14
    //            (0xE4), 1, 0x1D,    //GATE CTRL: 240/8-1 = 29
    //            (0xC2), 2, 0x01, 0xFF,      //VDVVRHEN: VDV and VRH Command Enable [01 FF]
    //            (0xC3), 1, 0x11,    //VRHS: VRH Set VAP=4.4, VAN=-4.4 [0B]
    //            (0xC4), 1, 0x20,    //VDVS: VDV Set [20]
    //            (0xC6), 1, 0x0F,    //FRCTRL2: Frame Rate control in normal mode [0F]
    //            (0xD0), 2, 0xA4, 0xA1,      //PWCTRL1: Power Control 1 [A4 A1]
    (0xE0), 14, 0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19,     //PVGAMCTRL: Positive Voltage Gamma control
    (0xE1), 14, 0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19,     //NVGAMCTRL: Negative Voltage Gamma control
    0x11, 0,            //Sleep Out
    TFTLCD_DELAY8, 150,
    0x29, 0,            //Display On
};
static const uint8_t ST7789_regValues[] PROGMEM = {
    0x01, 0,            //Soft Reset
    TFTLCD_DELAY8, 150,  // .kbv will power up with ONLY reset, sleep out, display on
    0x28, 0,            //Display Off
    0x3A, 1, 0x55,      //Pixel read=565, write=565.
    (0xB2), 5, 0x0C, 0x0C, 0x00, 0x33, 0x33,    //PORCTRK: Porch setting [08 08 00 22 22] PSEN=0 anyway
    (0xB7), 1, 0x35,    //GCTRL: Gate Control [35]
    (0xBB), 1, 0x2B,    //VCOMS: VCOM setting VCOM=1.175 [20] VCOM=0.9
    (0xC0), 1, 0x1C,    //LCMCTRL: LCM Control [2C] was=14
    //            (0xE4), 1, 0x1D,    //GATE CTRL: 240/8-1 = 29
    (0xC2), 2, 0x01, 0xFF,      //VDVVRHEN: VDV and VRH Command Enable [01 FF]
    (0xC3), 1, 0x11,    //VRHS: VRH Set VAP=4.4, VAN=-4.4 [0B]
    (0xC4), 1, 0x20,    //VDVS: VDV Set [20]
    (0xC6), 1, 0x0F,    //FRCTRL2: Frame Rate control in normal mode [0F]
    (0xD0), 2, 0xA4, 0xA1,      //PWCTRL1: Power Control 1 [A4 A1]
    (0xE0), 14, 0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19,     //PVGAMCTRL: Positive Voltage Gamma control
    (0xE1), 14, 0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19,     //NVGAMCTRL: Negative Voltage Gamma control
    0x11, 0,            //Sleep Out
    TFTLCD_DELAY8, 150,
    0x29, 0,            //Display On
};
static const uint8_t ST7789_regValues_arcain6[] PROGMEM = {
    0x01, 0,            //Soft Reset
    TFTLCD_DELAY8, 150,  // .kbv will power up with ONLY reset, sleep out, display on
    0x28, 0,            //Display Off
    0x3A, 1, 0x55,      //Pixel read=565, write=565.
    (0xB2), 5, 0x0C, 0x0C, 0x00, 0x33, 0x33,    //PORCTRK: Porch setting [08 08 00 22 22] PSEN=0 anyway
    (0xB7), 1, 0x35,    //GCTRL: Gate Control [35]
    (0xBB), 1, 0x35,    //VCOMS: VCOM setting VCOM=??? [20] VCOM=0.9
    (0xC0), 1, 0x2C,    //LCMCTRL: LCM Control [2C]
    (0xC2), 2, 0x01, 0xFF,      //VDVVRHEN: VDV and VRH Command Enable [01 FF]
    (0xC3), 1, 0x13,    //VRHS: VRH Set VAP=???, VAN=-??? [0B]
    (0xC4), 1, 0x20,    //VDVS: VDV Set [20]
    (0xC6), 1, 0x0F,    //FRCTRL2: Frame Rate control in normal mode [0F]
    (0xCA), 1, 0x0F,    //REGSEL2 [0F]
    (0xC8), 1, 0x08,    //REGSEL1 [08]
    (0x55), 1, 0x90,    //WRCACE  [00]
    (0xD0), 2, 0xA4, 0xA1,      //PWCTRL1: Power Control 1 [A4 A1]
    (0xE0), 14, 0xD0, 0x00, 0x06, 0x09, 0x0B, 0x2A, 0x3C, 0x55, 0x4B, 0x08, 0x16, 0x14, 0x19, 0x20,     //PVGAMCTRL: Positive Voltage Gamma control
    (0xE1), 14, 0xD0, 0x00, 0x06, 0x09, 0x0B, 0x29, 0x36, 0x54, 0x4B, 0x0D, 0x16, 0x14, 0x21, 0x20,     //NVGAMCTRL: Negative Voltage Gamma control
    0x11, 0,            //Sleep Out
    TFTLCD_DELAY8, 150,
    0x29, 0,            //Display On
};

//#define tableNNNN ST7789_regValues_Adafruit
#define tableNNNN ST7789_regValues

void ST7789_kbv::begin(uint16_t ID)
{
    _lcd_ID = ID;
    uint8_t *p = (uint8_t *) tableNNNN;
    int16_t size = sizeof(tableNNNN);
    reset();
    while (size > 0) {
        uint8_t cmd = pgm_read_byte(p++);
        uint8_t len = pgm_read_byte(p++);
        if (cmd == TFTLCD_DELAY8) {
            delay(len);
            len = 0;
        } else {
            WriteCmd(cmd);
            for (uint8_t d = 0; d < len; d++) {
                uint8_t x = pgm_read_byte(p++);
                WriteDat(x);
            }
            FLUSH_IDLE;
        }
        size -= len + 2;
    }
    setRotation(0);             //PORTRAIT
}
