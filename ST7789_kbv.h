#ifndef ST7789_KBV_H_
#define ST7789_KBV_H_ 100

#define USE_MBED defined(__MBED__)
#define USE_NO_CS 

#if ARDUINO < 165
#define USE_GFX_KBV
#include "ADA_GFX_kbv.h"
#else
#include "Adafruit_GFX.h"
#endif

class ST7789_kbv : public Adafruit_GFX {

#if USE_MBED
public:
#if defined(TARGET_LPC1768)
  ST7789_kbv(PinName CS=p8, PinName RS=p9, PinName RST=p10);
#else
  ST7789_kbv(PinName CS=D10, PinName RS=D9, PinName RST=D8);
#endif
private:
    void write16_N(uint16_t color, int16_t n);
    SPI _spi;
    DigitalOut      _lcd_pin_cs, _lcd_pin_rs, _lcd_pin_reset;
#else
public:
	ST7789_kbv(uint16_t id = 0x7789, int w = 240, int h = 240, int offset = 0, int xorf = -1);
#endif
public:
	void     reset(void);                                       // you only need the constructor
	void     begin(uint16_t ID = 0x7789);                                       // you only need the constructor
	virtual void     drawPixel(int16_t x, int16_t y, uint16_t color);  // and these three
    void     pushCommand(uint16_t cmd, uint8_t * block, int8_t N);
	uint16_t color565(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }
	uint16_t readID(void); // { return 0x9341; }

	virtual void     fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	virtual void     drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) { fillRect(x, y, 1, h, color); }
	virtual void     drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) { fillRect(x, y, w, 1, color); }
	virtual void     fillScreen(uint16_t color)                                     { fillRect(0, 0, _width, _height, color); }
	virtual void     setRotation(uint8_t r);
	virtual void     invertDisplay(boolean i);

	uint16_t readReg(uint16_t reg, uint8_t idx = 0);            //note that this reads pairs of data bytes
	int16_t  readGRAM(int16_t x, int16_t y, uint16_t *block, int16_t w, int16_t h);
	uint16_t readPixel(int16_t x, int16_t y) { uint16_t color; readGRAM(x, y, &color, 1, 1); return color; }
	void     setAddrWindow(int16_t x, int16_t y, int16_t x1, int16_t y1);
	void     pushColors(uint16_t *block, int16_t n, bool first);
	void     pushColors(uint8_t *block, int16_t n, bool first);    //NEW
	void     pushColors(const uint8_t *block, int16_t n, bool first, bool bigend = false); //NEW
	void     vertScroll(int16_t top, int16_t scrollines, int16_t offset);

	protected:
        void pushColors_any(uint16_t cmd, uint8_t * block, int16_t n, bool first, uint8_t flags);

	private:
    uint8_t readReg8(uint8_t reg, uint8_t dat);
	uint8_t         _lcd_xor;
	uint16_t        _lcd_ID, __OFFSET;
};

// New color definitions.  thanks to Bodmer
#define TFT_BLACK       0x0000      /*   0,   0,   0 */
#define TFT_NAVY        0x000F      /*   0,   0, 128 */
#define TFT_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define TFT_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define TFT_MAROON      0x7800      /* 128,   0,   0 */
#define TFT_PURPLE      0x780F      /* 128,   0, 128 */
#define TFT_OLIVE       0x7BE0      /* 128, 128,   0 */
#define TFT_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define TFT_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define TFT_BLUE        0x001F      /*   0,   0, 255 */
#define TFT_GREEN       0x07E0      /*   0, 255,   0 */
#define TFT_CYAN        0x07FF      /*   0, 255, 255 */
#define TFT_RED         0xF800      /* 255,   0,   0 */
#define TFT_MAGENTA     0xF81F      /* 255,   0, 255 */
#define TFT_YELLOW      0xFFE0      /* 255, 255,   0 */
#define TFT_WHITE       0xFFFF      /* 255, 255, 255 */
#define TFT_ORANGE      0xFDA0      /* 255, 180,   0 */
#define TFT_GREENYELLOW 0xB7E0      /* 180, 255,   0 */
#define TFT_PINK        0xFC9F

#endif
