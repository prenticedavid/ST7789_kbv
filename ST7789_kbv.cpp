//to do: check MBED timing
//to do: read ID at faster SPI speeds

#include "ST7789_kbv.h"

#if defined(__MBED__)
#include "serial_mbed.h"

ST7789_kbv::ST7789_kbv(PinName CS, PinName RS, PinName RST)
    : _lcd_pin_rs(RS), _lcd_pin_cs(CS), _lcd_pin_reset(RST), _spi(D11, D12, D13), Adafruit_GFX(240, 240)
{
}

#elif defined(__CC_ARM) || defined(__CROSSWORKS_ARM)
#include "serial_keil.h"

ST7789_kbv::ST7789_kbv():Adafruit_GFX(240, 240)
{
}
#else
#include "serial_kbv.h"

ST7789_kbv::ST7789_kbv():Adafruit_GFX(240, 240)
{
}
#endif

static uint8_t done_reset;

void ST7789_kbv::reset(void)
{
    done_reset = 1;
    INIT();
    CS_IDLE;
    RESET_IDLE;
    wait_ms(50);
    RESET_ACTIVE;
    wait_ms(100);
    RESET_IDLE;
    wait_ms(100);
}

void ST7789_kbv::pushCommand(uint16_t cmd, uint8_t * block, int8_t N)
{
    CS_ACTIVE;
    WriteCmd(cmd);
    write8_block(block, N);
    FLUSH_IDLE;
}

static uint32_t readRegister(uint8_t reg)
{
    uint32_t ret;
	uint8_t bits = 8;
	if (reg == 4) bits = 25;
	if (reg == 9) bits = 33;
    CS_ACTIVE;
    WriteCmd(reg);
    CD_DATA;                    //
	SDIO_INMODE();
    ret = readbits(bits);
    CS_IDLE;
	SDIO_OUTMODE();
    return ret;	
}

uint16_t ST7789_kbv::readID(void)                          //{ return 0x9341; }
{
    if (!done_reset) reset();
//    uint32_t ret = readRegister(4);
//	return ret == 0x858552 ? 0x7789 : ret;
    return 0x7789;
}

uint16_t ST7789_kbv::readReg(uint16_t reg, uint8_t idx)
{
    uint32_t ret = readRegister(reg);
	if (idx == 1) ret >>= 16;
	return ret;
}

int16_t ST7789_kbv::readGRAM(int16_t x, int16_t y, uint16_t * block, int16_t w, int16_t h)
{
	uint8_t r, g, b;
	int16_t n = w * h;    // we are NEVER going to read > 32k pixels at once
	uint8_t colmod = 0x66;
	pushCommand(0x3A, &colmod, 1);
	setAddrWindow(x, y, x + w - 1, y + h - 1);
	CS_ACTIVE;
	WriteCmd(0x2E);
	CD_DATA;
	SDIO_INMODE();        // do this while CS is Active

	r = readbits(8 + 1);	  //(8) for ILI9163, (9) for ST7735
	while (n-- > 0) {
		r = readbits(8);
		g = readbits(8);
		b = readbits(8);
		*block++ = color565(r, g, b);
	}
	CS_IDLE;
	SDIO_OUTMODE();      //do this when CS is Idle
    setAddrWindow(0, 0, width() - 1, height() - 1);
	colmod = 0x05;
	pushCommand(0x3A, &colmod, 1);
    return 0;
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
    uint8_t d[3] = {0x27, 0x00, 0x10};
    if (rotation & 2) d[1] = 0x0A;
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
    write16(color);
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
    CS_ACTIVE;
    WriteCmd(0x2C);
    if (h > w) { end = h; h = w; w = end; }
    while (h-- > 0) {
        write16_N(color, w);
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
        if (isbigend) xchg8(h);
        xchg8(l);
        if (!isbigend) xchg8(h);
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
    CS_ACTIVE;
    WriteCmd( 0x33);
    write16(top);        //TOP
    write16(scrollines); //VSA
    write16(bfa);        //BFA

    WriteCmd(0x37)
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

#define tableNNNN ST7789_regValues_Adafruit

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
            CS_ACTIVE;
            WriteCmd(cmd);
            for (uint8_t d = 0; d < len; d++) {
                uint8_t x = pgm_read_byte(p++);
                xchg8(x);
            }
            FLUSH_IDLE;
        }
        size -= len + 2;
    }
    setRotation(0);             //PORTRAIT
}
