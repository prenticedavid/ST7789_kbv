//No, calling begin(), beginTransaction(4MHz, ...) does not fix C7789 speed
//A7789: edit Adafruit_ST77xx.cpp for SPI_DEFAULT_FREQ
//C7789: edit Fast_ILI9341.h / Arduino_Fast_ST7789.h for SPI_DEFAULT_FREQ
#define MAX_SPICLOCK 8000000  //Saleae must be <= 8MHz. 9341 will work up to 42MHz

// Edit for appropriate pins

#if defined(ARDUINO_RASPBERRY_PI_PICO)
#define TFT_CS  17 //21   //SPI control pins. Hardware MOSI, SCK
#define TFT_DC  7 //19
#define TFT_RST 6 //18
#elif defined(__AVR_AVR128DA48__) //Curiosity
#define TFT_CS  7   //PA7 SPI control pins. Hardware MOSI, SCK
#define TFT_DC  9   //PB1
#define TFT_RST 8   //PB0
#elif defined(__AVR_AVR128DB48__) //Curiosity
#define TFT_CS  7   //PA7 SPI control pins. Hardware MOSI, SCK
#define TFT_DC  17  //PC1
#define TFT_RST 16  //PC0
#elif defined(ESP32)
#define TFT_CS  5
#define TFT_DC  13
#define TFT_RST 12
#define SD_CS 17
#elif defined(ESP8266)
#define TFT_CS  D10
#define TFT_DC  D9
#define TFT_RST D8
#define SD_CS   D4
#else
#define TFT_CS  10   //SPI control pins. Hardware MOSI, SCK
#define TFT_DC  9
#define TFT_RST 8
#endif

#if 0
#elif USE_TFT_LIB == 0x0000   //Parallel.  Uno, Due, ESP32, ...
#include <MCUFRIEND_kbv.h>
MCUFRIEND_kbv lcd;  //control pins are fixed for shield
#define TFT_BEGIN()       {uint16_t ID = lcd.readID(); if (ID == 0xD3D3) ID = 0x6814;lcd.begin(ID); }
#define DRAWIMAGE(x, y, w, h, colors) { \
        lcd.setAddrWindow(x, y, x + w - 1, y + h - 1); \
        lcd.pushColors(colors, w * h, 1); \
    }

#elif USE_TFT_LIB == 0xE8266  //ESP8266, ESP32, STM32, Pico. Par + SPI
#include <TFT_eSPI.h>
TFT_eSPI lcd;  //control pins are in User_Setup.h
#define TFT_BEGIN()       { lcd.begin(); lcd.setSwapBytes(true); }
#define DRAWIMAGE(x, y, w, h, pixels) lcd.pushImage(x, y, w, h, pixels)

#elif (USE_TFT_LIB & 0xFFFF0000) == 0xA0000   // Adafruit family
#if USE_TFT_LIB == 0xA7789   //Uno, Due, ESP, ...
#include <Adafruit_ST7789.h>
// edit Adafruit_ST77xx.cpp for SPI_DEFAULT_FREQ
SPISettings settings7789(MAX_SPICLOCK, MSBFIRST, SPI_MODE3);
Adafruit_ST7789 lcd(TFT_CS, TFT_DC, TFT_RST); //
//#define TFT_BEGIN()       { lcd.init(SCR_WD, SCR_HT, SPI_MODE3); SPI.beginTransaction(settings7789); }
#define TFT_BEGIN()       { lcd.init(SCR_WD, SCR_HT, SPI_MODE3); }
#elif USE_TFT_LIB == 0xA7796   //Uno, Due, ESP, ...
#include <Adafruit_ST7796S_kbv.h>
Adafruit_ST7796S_kbv lcd(TFT_CS, TFT_DC, TFT_RST); //
#define TFT_BEGIN()       { lcd.begin(MAX_SPICLOCK); }
#elif USE_TFT_LIB == 0xA9341   //Uno, Due, ESP, ...
#include <Adafruit_ILI9341.h>
Adafruit_ILI9341 lcd(TFT_CS, TFT_DC, TFT_RST); //
#define TFT_BEGIN()       { lcd.begin(MAX_SPICLOCK); }
#endif
#define DRAWIMAGE(x, y, w, h, pixels) lcd.drawRGBBitmap(x, y, pixels, w, h)

#elif (USE_TFT_LIB & 0xFFFF0000) == 0xB0000   // my SPI family
#include <ST7789_kbv.h>
#if USE_TFT_LIB == 0xB7789   //SPI.  Uno, Due, ESP32, ...
ST7789_kbv lcd(0x7789, 240, 320);  //control pins are fixed in library
#elif USE_TFT_LIB == 0xB7796   //SPI.  Uno, Due, ESP32, ...
ST7789_kbv lcd(0x7796, 320, 480);  //control pins are fixed in library
#elif USE_TFT_LIB == 0xB9341   //SPI.  Uno, Due, ESP32, ...
ST7789_kbv lcd(0x9341, 240, 320);  //control pins are fixed in library
#elif USE_TFT_LIB == 0xB9488   //SPI.  Uno, Due, ESP32, ...
ST7789_kbv lcd(0x9488, 320, 480);  //control pins are fixed in library
#else
#error ST7789_kbv device
#endif
#define TFT_BEGIN()       { lcd.begin(lcd.readID()); }
#define DRAWIMAGE(x, y, w, h, colors) { \
        lcd.setAddrWindow(x, y, x + w - 1, y + h - 1); \
        lcd.pushColors(colors, w * h, 1); \
    }

#elif (USE_TFT_LIB & 0xFFFF0000) == 0xC0000   // Pawel's Fast family
#if USE_TFT_LIB == 0xC7789   //cbm80amiga.  (need COMPATIBILITY_MODE for Due)
#include <Arduino_ST7789_Fast.h>
SPISettings settings7789(MAX_SPICLOCK, MSBFIRST, SPI_MODE3);
Arduino_ST7789 lcd(TFT_DC, TFT_RST, TFT_CS); //
//#define TFT_BEGIN()       { lcd.init(SCR_WD, SCR_HT); SPI.beginTransaction(settings7789); }
#define TFT_BEGIN()       { lcd.init(SCR_WD, SCR_HT); }
#elif USE_TFT_LIB == 0xC9341   //cbm80amiga.  (need COMPATIBILITY_MODE for Due)
#include <ILI9341_Fast.h>
ILI9341 lcd(TFT_DC, TFT_RST, TFT_CS); //
#define TFT_BEGIN()       { lcd.begin(); }
#endif
#define DRAWIMAGE(x, y, w, h, pixels) lcd.drawImage(x, y, w, h, pixels)

#elif USE_TFT_LIB == 0xD9341   //Due and Uno
#include <ILI9341_due.h>
#include "fonts\SystemFont5x7.h"
ILI9341_due lcd(TFT_CS, TFT_DC, TFT_RST);
#define TFT_BEGIN()       { lcd.begin(); \
                            lcd.setTextLetterSpacing(1); lcd.setFont(SystemFont5x7); }
#define DRAWIMAGE(x, y, w, h, colors) { \
        lcd.setAddrWindow(x, y, x + w - 1, y + h - 1); \
        lcd.pushColors(colors, 0, w * h); \
    }
#define setTextSize(x) setTextScale(x)
#define setCursor(x, y) cursorToXY(x, y)

#elif (USE_TFT_LIB & 0xFFFF0000) == 0xF0000   //
#if USE_TFT_LIB == 0xF7781   //
#include <PDQ_GFX.h>    // IDE v1.7.8 needs this for INCDIR
#include "PDQ_ST7781_config.h"
#include <PDQ_ST7781.h>
PDQ_ST7781 lcd; //
#elif USE_TFT_LIB == 0xF9341   //
#include <SPI.h>
#include <PDQ_GFX.h>    // IDE v1.7.8 needs this for INCDIR
#include "PDQ_ILI9341_config.h"
#include <PDQ_ILI9341.h>
PDQ_ILI9341 lcd; //
#endif
#define TFT_BEGIN()       { lcd.begin(); }
#define DRAWIMAGE(x, y, w, h, colors) { \
        lcd.setAddrWindow(x, y, x + w - 1, y + h - 1); \
        uint16_t *p = colors; \
        for (int cnt = w * h; cnt--; ) lcd.pushColor(*p++); \
    }

#elif (USE_TFT_LIB & 0xFFFF0000) == 0xF30000   // Teensy T3 family
#if USE_TFT_LIB == 0xF39341   //Teensy
#include <ILI9341_t3.h>
ILI9341_t3 lcd(TFT_CS, TFT_DC, TFT_RST); //
#elif USE_TFT_LIB == 0xF39488   //Teensy
#include <ILI9488_t3.h>
ILI9488_t3 lcd(&SPI, TFT_CS, TFT_DC, TFT_RST, 11, 13, 12); //
#endif
#define TFT_BEGIN()       { lcd.begin(); }
#define DRAWIMAGE(x, y, w, h, colors) { lcd.writeRect(x, y, w, h, (uint16_t*)(colors)); }

#endif
