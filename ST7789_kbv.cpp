//to do: check MBED timing
//to do: read ID at faster SPI speeds

#include "ST7789_kbv.h"
#include <SPI.h>

#define USE_9BIT  0
#define USE_SPI   1
#define USE_ID    0x7735

#if defined(ESP32)
#define RESET_PIN 12
#define CD_PIN    13
#define CS_PIN    4 //5 //4
#define MOSI_PIN  23
#define SCK_PIN   18
#elif defined(ESP8266)
#define RESET_PIN D8
#define CD_PIN    D9
#define CS_PIN    D10
#define MOSI_PIN  D11
#define SCK_PIN   D13
#else
#define RESET_PIN 8
#define CD_PIN    9
#ifdef USE_NO_CS
#define CS_PIN    7
#else
#define CS_PIN    10
#endif
#define MOSI_PIN  11
#define SCK_PIN   13
#endif

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

#if 0
#elif USE_ID == 0x7735
#define tableNNNN table7735S
#define TFTWIDTH  128
#define TFTHEIGHT 160
#define USE_666   0       //565 or 666
#elif USE_ID == 0x7789
#define tableNNNN ST7789_regValues
#define TFTWIDTH  240
#define TFTHEIGHT 240
#define USE_666   0       //565 or 666
#elif USE_ID == 0x9163
#define tableNNNN table9163C
#define TFTWIDTH  128
#define TFTHEIGHT 128
#define USE_666   0       //565 or 666
#elif USE_ID == 0x9341
#define tableNNNN ILI9341_regValues_2_4
#define TFTWIDTH  240
#define TFTHEIGHT 320
#define USE_666   0       //565 or 666
#elif USE_ID == 0x9481
#define tableNNNN ILI9481_RGB_regValues
#define TFTWIDTH  320
#define TFTHEIGHT 480
#define USE_666   1       //always 666
#elif USE_ID == 0x9488
#define tableNNNN ILI9488_regValues_kbv
#define TFTWIDTH  320
#define TFTHEIGHT 480
#define USE_666   1       //always 666
#endif

static SPISettings settings(8000000, MSBFIRST, SPI_MODE3); //8MHz is max for Saleae. 12MHz is max for ILI9481

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
#if USE_SPI
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

static inline void writeColor(uint16_t color, uint32_t n)
{
    // alter this function for 666 format controllers
#if USE_666
    uint8_t r = color >> 8, g = (color >> 3), b = color << 3;
    while (n-- > 0) {
        WriteDat(r);
        WriteDat(g);
        WriteDat(b);
    }
#else
    uint8_t h = (color >> 8), l = color;
    while (n--) {
        WriteDat(h);
        WriteDat(l);
    }
#endif
}

ST7789_kbv::ST7789_kbv(uint16_t id, int w, int h, int offset):Adafruit_GFX(w, h)
{
    _lcd_ID = id;
    __OFFSET = offset;
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
//    return USE_ID;
    return _lcd_ID ? _lcd_ID : USE_ID;
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
#if USE_ID == 0x9481
        case 0:
            mac = 0x0A;
            break;
        case 1:        //LANDSCAPE 90 degrees
            mac = 0x28;
            break;
        case 2:
            mac = 0x09;
            break;
        case 3:
            mac = 0x2B;
            break;
#else
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
#endif
    }
    mac ^= _lcd_xor;
    pushCommand(0x36, &mac, 1);
#if USE_ID == 0x7789
    uint8_t d[3] = {0x27, 0x00, 0x10}; //regular 240x320
    if (HEIGHT == 240 && rotation & 2) d[1] = 0x0A;     //fix GateScan on 240x240 panel
    pushCommand(0xE4, d, 3);
#endif
}

void ST7789_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // ILI934X just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
	if (rotation == 0) y += __OFFSET;
	if (rotation == 1) x += __OFFSET;
    CS_ACTIVE;
    WriteCmd(0x2A);
    write16(x);
//    write16(x);
    WriteCmd(0x2B);
    write16(y);
//    write16(y);
    WriteCmd(0x2C);
    writeColor(color, 1);
    FLUSH_IDLE;
}

void ST7789_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
	if (rotation == 0) y += __OFFSET, y1 += __OFFSET;
	if (rotation == 1) x += __OFFSET, x1 += __OFFSET;
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
    CS_ACTIVE;
	if (first) {
        WriteCmd(cmd);
    }
#if USE_666 == 0
    if (!isconst) {
        uint16_t *block16 = (uint16_t*)block;
        int i = n;
        if (!isbigend) while (i--) { uint16_t color = *block16; *block16++ = (color >> 8)|(color << 8); }
        CD_DATA;
        SPI.transfer(block, n * 2);
    } else
#endif
    while (n-- > 0) {
        if (isconst) {
            h = pgm_read_byte(block++);
            l = pgm_read_byte(block++);
        } else {
            h = (*block++);
            l = (*block++);
        }
        uint16_t color;
        if (isbigend) color = (h << 8) | l;
        else color = (l << 8) | h;
        writeColor(color, 1);
    }
	FLUSH_IDLE;
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
    if (rotation == 0 || rotation == 1) top += __OFFSET;
    int16_t bfa = HEIGHT + __OFFSET - top - scrollines;  // Ilitek checks valid
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
static const uint8_t reset_off[] PROGMEM = {
    0x01, 0,            //Soft Reset
    TFTLCD_DELAY8, 150,  // .kbv will power up with ONLY reset, sleep out, display on
    0x28, 0,            //Display Off
};
static const uint8_t wake_on[] PROGMEM = {
    0x11, 0,            //Sleep Out
    TFTLCD_DELAY8, 150,
#if USE_666
    0x3A, 1, 0x66,      //Pixel read=666, write=666.
#else
    0x3A, 1, 0x55,      //Pixel read=565, write=565.
#endif
    0x29, 0,            //Display On
};

static const uint8_t PROGMEM table7735S[] = {
            //  (COMMAND_BYTE), n, data_bytes....
            0xB1, 3, 0x01, 0x2C, 0x2D,  // [05 3C 3C] FRMCTR1 if GM==11
            0xB2, 3, 0x01, 0x2C, 0x2D,  // [05 3C 3C]
            0xB3, 6, 0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D, // [05 3C 3C 05 3C 3C]
            0xB4, 1, 0x07,              // [07] INVCTR Column inversion
            //ST7735XR Power Sequence
            0xC0, 3, 0xA2, 0x02, 0x84,  // [A8 08 84] PWCTR1
            0xC1, 1, 0xC5,              // [C0]
            0xC2, 2, 0x0A, 0x00,        // [0A 00]
            0xC3, 2, 0x8A, 0x2A,        // [8A 26]
            0xC4, 2, 0x8A, 0xEE,        // [8A EE]
            0xC5, 1, 0x0E,              // [05] VMCTR1 VCOM
            };

static const uint8_t PROGMEM table9163C[] = {
            //  (COMMAND_BYTE), n, data_bytes....
            0x26, 1, 0x02,       // [01] GAMMASET use CURVE=1, 2, 4, 8
            0xB1, 2, 0x08, 0x02, // [0E 14] FRMCTR1 if GM==011 61.7Hz
            0xB4, 1, 0x07,       // [02] INVCTR
            0xB8, 1, 0x01,       // [00] GSCTRL
            0xC0, 2, 0x0A, 0x02, // [0A 05] PWCTR1 if LCM==10
            0xC1, 1, 0x02,       // [07] PWCTR2
            0xC5, 2, 0x50, 0x63, // [43 4D] VMCTR1
            0xC7, 1, 0x00,       // [40] VCOMOFFS
            };

static const uint8_t ST7789_regValues[] PROGMEM = {
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
//    (0xE0), 14, 0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19,     //PVGAMCTRL: Positive Voltage Gamma control
//    (0xE1), 14, 0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19,     //NVGAMCTRL: Negative Voltage Gamma control
};

static const uint8_t ILI9341_regValues_2_4[] PROGMEM = {   // BOE 2.4"
    0xCF, 3, 0x00, 0x81, 0x30,  //Power Control B [00 81 30]
    0xED, 4, 0x64, 0x03, 0x12, 0x81,    //Power On Seq [55 01 23 01]
    0xE8, 3, 0x85, 0x10, 0x78,  //Driver Timing A [04 11 7A]
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,      //Power Control A [39 2C 00 34 02]
    0xF7, 1, 0x20,      //Pump Ratio [10]
    0xEA, 2, 0x00, 0x00,        //Driver Timing B [66 00]
    0xB1, 2, 0x00, 0x1B,        //Frame Control [00 1B]
    0xB6, 3, 0x0A, 0xA2, 0x27, //Display Function [0A 82 27 XX]    .kbv REV=1,SS=1
    0xF6, 1, 0x01,      //Interface Control [01 00 00] .kbv BGR=0
    0xB4, 1, 0x00,      //Inversion Control [02] .kbv NLA=1, NLB=1, NLC=1
    0xC0, 1, 0x21,      //Power Control 1 [26]
    0xC1, 1, 0x11,      //Power Control 2 [00]
    0xC5, 2, 0x3F, 0x3C,        //VCOM 1 [31 3C]
    0xC7, 1, 0xB5,      //VCOM 2 [C0]
//    0x36, 1, 0x48,      //Memory Access [00]
//    0xF2, 1, 0x00,      //Enable 3G [02]
    0x26, 1, 0x01,      //Gamma Set [01]
//    0xE0, 15, 0x0f, 0x26, 0x24, 0x0b, 0x0e, 0x09, 0x54, 0xa8, 0x46, 0x0c, 0x17, 0x09, 0x0f, 0x07, 0x00,
//    0xE1, 15, 0x00, 0x19, 0x1b, 0x04, 0x10, 0x07, 0x2a, 0x47, 0x39, 0x03, 0x06, 0x06, 0x30, 0x38, 0x0f,
};

static const uint8_t ILI9481_RGB_regValues[] PROGMEM = {    // 320x480
    0xB0, 1, 0x00,
    0xD0, 3, 0x07, 0x41, 0x1D,  // SETPOWER [00 43 18]
    0xD1, 3, 0x00, 0x2B, 0x1F,  // SETVCOM  [00 00 00] x0.900, x1.32
    0xD2, 2, 0x01, 0x11,        // SETNORPOW for Normal Mode [01 22]
    0xC0, 6, 0x00, 0x3B, 0x00, 0x02, 0x11, 0x00,     //SETPANEL [10 3B 00 02 11]
    0xC5, 1, 0x03,      //SETOSC Frame Rate [03]
    0xC6, 1, 0x80,      //SETRGB interface control
//    0xC8, 12, 0x00, 0x14, 0x33, 0x10, 0x00, 0x16, 0x44, 0x36, 0x77, 0x00, 0x0F, 0x00,
    0xF3, 2, 0x40, 0x0A,
    0xF0, 1, 0x08,
    0xF6, 1, 0x84,
    0xF7, 1, 0x80,
    0x0C, 2, 0x00, 0x55, //RDCOLMOD
    0xB4, 1, 0x00,      //SETDISPLAY
    //          0xB3, 4, 0x00, 0x01, 0x06, 0x01,  //SETGRAM simple example
    0xB3, 4, 0x00, 0x01, 0x06, 0x30,  //jpegs example
};

const uint8_t PROGMEM ILI9488_regValues_kbv[] = {
    //  (COMMAND_BYTE), n, data_bytes....
    0xC0, 2, 0x10, 0x10,        //Power Control 1 [0E 0E]
    0xC1, 1, 0x41,      //Power Control 2 [43]
    0xC5, 4, 0x00, 0x22, 0x80, 0x40,    //VCOM  Control 1 [00 40 00 40]
    0x36, 1, 0x68,      //Memory Access [00]
    0xB0, 1, 0x00,      //Interface     [00]
    0xB1, 2, 0xB0, 0x11,        //Frame Rate Control [B0 11]
    0xB4, 1, 0x02,      //Inversion Control [02]
    0xB6, 3, 0x02, 0x22, 0x3B,  // Display Function Control [02 02 3B] .kbv SS=1, NL=480
    0xB7, 1, 0xC6,      //Entry Mode      [06]
    0x3A, 1, 0x66,      //Interlace Pixel Format [XX]
    0xF7, 4, 0xA9, 0x51, 0x2C, 0x82,    //Adjustment Control 3 [A9 51 2C 82]
};

static void init_table(const void *table, int16_t size)
{
    uint8_t *p = (uint8_t *) table;
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
}

void ST7789_kbv::begin(uint16_t ID)
{
    const uint8_t *table;
    int size;
    _lcd_ID = ID;
    _lcd_xor = 0;
    __OFFSET = 0;
    switch (ID) {
	case 0x9101:
	    __OFFSET = 32;
	    _lcd_xor = 0xD0;
        goto common_7735;
    case 0x7735:
        _lcd_xor = 0xD8;
common_7735:
        table = table7735S;
        size = sizeof(table7735S);
        break;
    case 0x7789:
        table = ST7789_regValues;
        size = sizeof(ST7789_regValues);
        break;
	case 0x9162:
	    __OFFSET = 32;
	    _lcd_xor = 0x90;
        goto common_9163;
    case 0x9163:
        _lcd_xor = 0x40;
common_9163:
        table = table9163C;
        size = sizeof(table9163C);
        break;
    case 0x9341:
        table = ILI9341_regValues_2_4;
        size = sizeof(ILI9341_regValues_2_4);
        break;
    case 0x9481:
        table = ILI9481_RGB_regValues;
        size = sizeof(ILI9481_RGB_regValues);
        break;
    case 0x9488:
        table = ILI9488_regValues_kbv;
        size = sizeof(ILI9488_regValues_kbv);
        break;
    }
    reset();
		init_table(&reset_off, sizeof(reset_off));
//        init_table(tableNNNN, sizeof(tableNNNN));    
        init_table(table, size);    
		init_table(&wake_on, sizeof(wake_on));   //can change PIXFMT
    setRotation(0);             //PORTRAIT
}
