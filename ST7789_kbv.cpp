//to do: check MBED timing
//to do: read ID at faster SPI speeds

#include "ST7789_kbv.h"

#define USE_MODE  SPI_MODE3
#define USE_9BIT  0
#define USE_SPI   1
#define USE_666   0       //565 or 666
#define USE_ID    0x7735
#define SUPPORT_1351      //inserts is1351 conditionals

#include "ST7789_serial.h"
//#include "serial_kbv.h"

static bool is1351, use_666;

static inline void writeColor(uint16_t color, uint32_t n)
{
    // alter this function for 666 format controllers
    if (use_666) write24_N(color, n);  //SSD1351 has color in bits 5..0
    else write16_N(color, n);
}

ST7789_kbv::ST7789_kbv(uint16_t id, int w, int h, int offset, int xorf): Adafruit_GFX(w, h)
{
    _lcd_ID = id;
    __OFFSET = offset;
    _lcd_xor = xorf;
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

uint32_t ST7789_kbv::readReg32(uint16_t reg)
{
    uint32_t ret;
    uint8_t bits = 8;
    if (reg == 4) bits = 25;
    if (reg == 9) bits = 33;
    WriteCmd(reg);
    SDIO_INMODE();
    ret = readbits(bits);
    SDIO_OUTMODE();
    FLUSH_IDLE;
    return ret;	
}

uint8_t ST7789_kbv::readcommand8(uint8_t reg, uint8_t idx) //this is the same as Adafruit_ILI9341
{
    uint8_t SPIREAD_CMD = (_lcd_ID == 0x9341) ? 0xD9 : 0xFB;
    uint8_t SPIREAD_EN  = (_lcd_ID == 0x9341) ? 0x10 : 0x80;
    uint8_t off = 0;
    uint8_t ret;
    bool is_sekret = (_lcd_ID == 0x9341 || _lcd_ID == 0x9488);
    SPIREAD_EN |= idx;
    if (is_sekret) pushCommand(SPIREAD_CMD, &SPIREAD_EN, 1); //special for 9341, 9488
    WriteCmd(reg);
    ret = xchg8(0xFF);                       //only safe to read @ SCK=16MHz
    if (is_sekret) pushCommand(SPIREAD_CMD, &off, 1);        //ILI9488 MUST disable afterwards
    return ret;
}

uint16_t ST7789_kbv::readID(void)                          //{ return 0x9341; }
{
    if (!done_reset) reset();
    return _lcd_ID ? _lcd_ID : USE_ID;
}

uint16_t ST7789_kbv::readReg(uint16_t reg, uint8_t idx)
{
    if (_lcd_ID != 0x9341 && _lcd_ID != 0x9488) return 0x1234;
    uint8_t h, l;
    idx <<= 1;
    h = readcommand8(reg, idx);
    l = readcommand8(reg, idx + 1);
    return (h << 8) | l;
}

int16_t ST7789_kbv::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
    uint8_t r, g, b;
    int16_t n = w * h;    // we are NEVER going to read > 32k pixels at once
#if defined(USE_NO_CS)
    // might try ILI9341 without CS and WEMODE=0
    return -1;
#endif
    setAddrWindow(x, y, x + w - 1, y + h - 1);
    WriteCmd(0x2E);
    if (_lcd_ID == 0x9341 || _lcd_ID == 0x9488) {
        r = xchg8(0xFF);
        while (n-- > 0) {
            r = xchg8(0xFF);
            g = xchg8(0xFF);
            b = xchg8(0xFF);
            *block++ = color565(r, g, b);
        }
    } else {
        SDIO_INMODE();
        readbits(8 + (_lcd_ID == 0x7735));
        while (n-- > 0) {
            r = readbits(8);
            g = readbits(8);
            b = readbits(8);
            *block++ = color565(r, g, b);
        }
        SDIO_OUTMODE();
    }
    FLUSH_IDLE;
    setAddrWindow(0, 0, width() - 1, height() - 1);
    return 0;
}

static const uint8_t PROGMEM mactable[] = {
    0x08, 0x68, 0xD8, 0xB8,  //MIPI style
    0x08, 0x2A, 0x1B, 0x39,  //9481
    0x74, 0x77, 0x66, 0x65,  //1351 565
    0xB4, 0xB7, 0xA6, 0xA5,  //1351 666
};

void ST7789_kbv::setRotation(uint8_t r)
{
    uint8_t mac, madctl = 0x36, ofs = 0;;
    Adafruit_GFX::setRotation(r & 3);
    if (_lcd_ID == 0x9481) ofs = 4;
    if (is1351) {
        madctl = 0xA0, ofs = 8, _MC = 0x15, _MP = 0x75, _MW = 0x5C;
        if (use_666) ofs += 4;
        if (rotation & 1) _MC = 0x75, _MP = 0x15;
    }
    mac = pgm_read_byte(mactable + ofs + rotation);
    mac ^= _lcd_xor;
    pushCommand(madctl, &mac, 1);
    vertScroll(0, HEIGHT, 0);
    if (_lcd_ID == 0x7789 && __OFFSET == 0) {
        uint8_t d[3] = {0x27, 0x00, 0x10};              //always use 320 scan lines
        if (HEIGHT == 240 && rotation & 2) d[1] = 0x0A; //adjust SCS on 240x240 panel
        pushCommand(0xE4, d, 3);                        //
    }
}

void ST7789_kbv::drawPixel(int16_t x, int16_t y, uint16_t color)
{
    // ILI934X just plots at edge if you try to write outside of the box:
    if (x < 0 || y < 0 || x >= width() || y >= height())
        return;
    if (rotation == 0) y += __OFFSET;
    if (rotation == 1) x += __OFFSET;
    bool is_9488 = (_lcd_ID == 0x9488);
    if (is1351) x |= x << 8, y |= y << 8; //squeeze two 8-bits
    WriteCmd(_MC);
    write16(x);
    if (is_9488) write16(x);
    WriteCmd(_MP);
    write16(y);
    if (is_9488) write16(y);
    WriteCmd(_MW);
    writeColor(color, 1);
    FLUSH_IDLE;
}

void ST7789_kbv::setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1)
{
    if (rotation == 0) y += __OFFSET, y1 += __OFFSET;
    if (rotation == 1) x += __OFFSET, x1 += __OFFSET;
    if (is1351) x1 |= x << 8, y1 |= y << 8; //squeeze two 8-bits
    WriteCmd(_MC);
    if (!is1351) write16(x); //1351 squeezes into x1
    write16(x1);
    WriteCmd(_MP);
    if (!is1351) write16(y);
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
    WriteCmd(_MW);
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
    CS_ACTIVE;      //subsequent calls
    if (first) {
        WriteCmd(cmd);
    }
    if (!isconst && !use_666) {
        uint16_t *block16 = (uint16_t*)block;
        int i = n;
        if (!isbigend) while (i--) {
                uint16_t color = *block16;
                *block16++ = (color >> 8) | (color << 8);
            }
        write8_block(block, n * 2);
    } else
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
    pushColors_any(_MW, (uint8_t *)block, n, first, 0);
}
void ST7789_kbv::pushColors(uint8_t * block, int16_t n, bool first)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, 2);   //regular bigend
}
void ST7789_kbv::pushColors(const uint8_t * block, int16_t n, bool first, bool bigend)
{
    pushColors_any(_MW, (uint8_t *)block, n, first, bigend ? 3 : 1);
}

void ST7789_kbv::invertDisplay(bool i)
{
    uint8_t normal = (is1351) ? 0xA6 : 0x20;
    pushCommand(normal + i, NULL, 0);
}

void ST7789_kbv::vertScroll(int16_t top, int16_t scrollines, int16_t offset)
{
    if (is1351) {
        pushCommand(0xA1, (uint8_t*)&offset, 1);
        return;
    }
    if (rotation == 0 || rotation == 1) top += __OFFSET;
    int16_t bfa = HEIGHT + __OFFSET - top - scrollines;  // Ilitek checks valid
    int16_t vsp;
    if (_lcd_ID == 0x7789 && HEIGHT == 240) bfa += 80;
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
    0x29, 0,            //Display On
};

static const uint8_t PROGMEM table1351[] = {
    //  (COMMAND_BYTE), n, data_bytes....
    0xFD, 1, 0x12,       // set command lock
    0xFD, 1, 0xB1,       // set command lock
    0xAE, 0,             //display off
    0xB3, 1, 0xF1,       //clock freq=15, div=1
    0xCA, 1, 0x7F,       //multiplex = 128
    0xA0, 1, 0x74,       //remap  COL=1, SM=1, SS=1, RGB=1, GS=0, MV=0
    0x15, 2, 0x00, 0x7F, //SETCOL
    0x75, 2, 0x00, 0x7F, //SETROW
    0xA1, 1, 0x00,       //STARTLINE
    0xA2, 1, 0x00,       //DISPLAYOFFSET
    0xB5, 1, 0x00,       //SETGPIO
    0xAB, 1, 0x01,       //FUNCTIONSELECT internal
    0xB1, 1, 0x32,       //PRECHARGE
    0xBE, 1, 0x05,       //VCOMH
    0xA6, 0,             //NORMAL
    0xC1, 3, 0xC8, 0x80, 0xC8, //CONTRASTABC
    0xC7, 1, 0x07,       //CONTRASTMASTER [0F]
    0xB4, 3, 0xA0, 0xB5, 0x55, //SETVSL
    0xB6, 1, 0x01,       //PRECHARGE2
    0xAF, 0,             //display on
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
    0x26, 1, 0x02,              // [01] GAMMASET use CURVE=1, 2, 4, 8
    0xB1, 2, 0x08, 0x02,        // [0E 14] FRMCTR1 if GM==011 61.7Hz
    0xB4, 1, 0x07,              // [02] INVCTR
    0xB8, 1, 0x01,              // [00] GSCTRL
    0xC0, 2, 0x0A, 0x02,        // [0A 05] PWCTR1 if LCM==10
    0xC1, 1, 0x02,              // [07] PWCTR2
    0xC5, 2, 0x50, 0x63,        // [43 4D] VMCTR1
    0xC7, 1, 0x00,              // [40] VCOMOFFS
};

static const uint8_t ST7789_regValues[] PROGMEM = {
    (0xB2), 5, 0x0C, 0x0C, 0x00, 0x33, 0x33,    //PORCTRK: Porch setting [08 08 00 22 22] PSEN=0 anyway
    (0xB7), 1, 0x35,            //GCTRL: Gate Control [35]
    (0xBB), 1, 0x2B,            //VCOMS: VCOM setting VCOM=1.175 [20] VCOM=0.9
    (0xC0), 1, 0x1C,            //LCMCTRL: LCM Control [2C] was=14
    //    (0xE4), 1, 0x27,    //GATE CTRL: NL=320/8-1 = 39
    (0xC2), 2, 0x01, 0xFF,      //VDVVRHEN: VDV and VRH Command Enable [01 FF]
    (0xC3), 1, 0x11,            //VRHS: VRH Set VAP=4.4, VAN=-4.4 [0B]
    (0xC4), 1, 0x20,            //VDVS: VDV Set [20]
    (0xC6), 1, 0x0F,            //FRCTRL2: Frame Rate control in normal mode [0F]
    (0xD0), 2, 0xA4, 0xA1,      //PWCTRL1: Power Control 1 [A4 A1]
    //    (0xE0), 14, 0xD0, 0x00, 0x05, 0x0E, 0x15, 0x0D, 0x37, 0x43, 0x47, 0x09, 0x15, 0x12, 0x16, 0x19,     //PVGAMCTRL: Positive Voltage Gamma control
    //    (0xE1), 14, 0xD0, 0x00, 0x05, 0x0D, 0x0C, 0x06, 0x2D, 0x44, 0x40, 0x0E, 0x1C, 0x18, 0x16, 0x19,     //NVGAMCTRL: Negative Voltage Gamma control
};

static const uint8_t ILI9341_regValues_2_4[] PROGMEM = {   // BOE 2.4"
    0xCF, 3, 0x00, 0x81, 0x30,  //Power Control B [00 81 30]
    0xED, 4, 0x64, 0x03, 0x12, 0x81,    //Power On Seq [55 01 23 01]
    0xE8, 3, 0x85, 0x10, 0x78,  //Driver Timing A [04 11 7A]
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,      //Power Control A [39 2C 00 34 02]
    0xF7, 1, 0x20,              //Pump Ratio [10]
    0xEA, 2, 0x00, 0x00,        //Driver Timing B [66 00]
    0xB1, 2, 0x00, 0x1B,        //Frame Control [00 1B]
    0xB6, 3, 0x0A, 0xA2, 0x27,  //Display Function [0A 82 27 XX]    .kbv REV=1,SS=1
    0xF6, 1, 0x01,              //Interface Control [01 00 00] .kbv BGR=0
    0xB4, 1, 0x00,              //Inversion Control [02] .kbv NLA=1, NLB=1, NLC=1
    0xC0, 1, 0x21,              //Power Control 1 [26]
    0xC1, 1, 0x11,              //Power Control 2 [00]
    0xC5, 2, 0x3F, 0x3C,        //VCOM 1 [31 3C]
    0xC7, 1, 0xB5,              //VCOM 2 [C0]
    //    0xF2, 1, 0x00,      //Enable 3G [02]
    0x26, 1, 0x01,              //Gamma Set [01]
    0xE0, 15, 0x0f, 0x26, 0x24, 0x0b, 0x0e, 0x09, 0x54, 0xa8, 0x46, 0x0c, 0x17, 0x09, 0x0f, 0x07, 0x00,
    0xE1, 15, 0x00, 0x19, 0x1b, 0x04, 0x10, 0x07, 0x2a, 0x47, 0x39, 0x03, 0x06, 0x06, 0x30, 0x38, 0x0f,
};

static const uint8_t ILI9481_RGB_regValues[] PROGMEM = {    // 320x480
    0xB0, 1, 0x00,
    0xD0, 3, 0x07, 0x41, 0x1D,  // SETPOWER [00 43 18]
    0xD1, 3, 0x00, 0x2B, 0x1F,  // SETVCOM  [00 00 00] x0.900, x1.32
    0xD2, 2, 0x01, 0x11,        // SETNORPOW for Normal Mode [01 22]
    0xC0, 6, 0x00, 0x3B, 0x00, 0x02, 0x11, 0x00,     //SETPANEL [10 3B 00 02 11]
    0xC5, 1, 0x03,              //SETOSC Frame Rate [03]
    0xC6, 1, 0x80,              //SETRGB interface control
    //    0xC8, 12, 0x00, 0x14, 0x33, 0x10, 0x00, 0x16, 0x44, 0x36, 0x77, 0x00, 0x0F, 0x00,
    0xF3, 2, 0x40, 0x0A,
    0xF0, 1, 0x08,
    0xF6, 1, 0x84,
    0xF7, 1, 0x80,
    0xB4, 1, 0x00,              //SETDISPLAY
    //          0xB3, 4, 0x00, 0x01, 0x06, 0x01,  //SETGRAM simple example
    0xB3, 4, 0x00, 0x01, 0x06, 0x30,  //jpegs example
};

const uint8_t PROGMEM ILI9488_regValues_kbv[] = {
    //  (COMMAND_BYTE), n, data_bytes....
    0xC0, 2, 0x10, 0x10,        //Power Control 1 [0E 0E]
    0xC1, 1, 0x41,              //Power Control 2 [43]
    0xC5, 4, 0x00, 0x22, 0x80, 0x40,    //VCOM  Control 1 [00 40 00 40]
    0x36, 1, 0x68,              //Memory Access [00]
    0xB0, 1, 0x00,              //Interface     [00]
    0xB1, 2, 0xB0, 0x11,        //Frame Rate Control [B0 11]
    0xB4, 1, 0x02,              //Inversion Control [02]
    0xB6, 3, 0x02, 0x22, 0x3B,  // Display Function Control [02 02 3B] .kbv SS=1, NL=480
    0xB7, 1, 0xC6,              //Entry Mode      [06]
    0x3A, 1, 0x66,              //Interlace Pixel Format [XX]
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
    use_666 = USE_666;
    _MC = 0x2A, _MP = 0x2B, _MW = 0x2C;  //default MIPI registers
    switch (ID) {
#ifdef SUPPORT_1351
        case 0x1351:
            table = table1351;
            size = sizeof(table1351);
            is1351 = true;
            break;
#endif
        case 0x7735:
            if (_lcd_xor == 0xFF) _lcd_xor = 0xD8;
            table = table7735S;
            size = sizeof(table7735S);
            break;
        case 0x7789:
            table = ST7789_regValues;
            size = sizeof(ST7789_regValues);
            break;
        case 0x9101:
            if (_lcd_xor == 0xFF) _lcd_xor = 0xD0;
            table = table7735S;
            size = sizeof(table7735S);
            break;
        case 0x9163:
            if (_lcd_xor == 0xFF) _lcd_xor = 0x40;
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
            use_666 = true;
            break;
        case 0x9488:
            table = ILI9488_regValues_kbv;
            size = sizeof(ILI9488_regValues_kbv);
            use_666 = true;
            break;
    }
    if (_lcd_xor == 0xFF) _lcd_xor = 0x00;   //default

    reset();
    if (!is1351) init_table(&reset_off, sizeof(reset_off));
    init_table(table, size);
    if (!is1351) init_table(&wake_on, sizeof(wake_on));
    if (!is1351) {
        uint8_t pixfmt = use_666 ? 0x66 : 0x55;
        pushCommand(0x3A, &pixfmt, 1);
    }
    setRotation(0);             //PORTRAIT
}
