// ILI9341 library example
// Amiga Boing Ball Demo
// (c) 2019-20 Pawel A. Hernik
// cbm80amiga has some excellent libraries on https://github.com/cbm80amiga
// YT video: https://youtu.be/KwtkfmglT-c

// I have always been fascinated by the Amiga Ball.
// and have adapted cbm80amiga's example sketch for
// diff geometry and diff libraries
// many libraries support GFX graphics.
// but constructor, BEGIN() and DRAWIMAGE() methods vary.
// it is fairly easy to adapt for another GFX library.
// Due, ESP, Xmega ball runs faster than you want.
// but Uno is not unreasonable

//USE_TFT_LIB value selects header, constructor, ...
#define USE_TFT_LIB 0xC9341 //0xA7789 //0xF39488 // e.g. Adafruit_ILI9341
//#define USE_TFT_LIB 0xE8266 // e.g. any TFT_eSPI=0xE8266. MCUFRIEND_kbv=0
#define SCR_WD      240  //alter ORIENTATION for Landscape
#define SCR_HT      320  //240x320: rot=2, inv=1 C7789. inv=0 A7789
#define SCR_INV     0    //ST7789 240x240 and 240x320 vary
#define ORIENTATION 0    //Portrait=0, Landscape=1, ...
#define USE_BIGBALL 0    //Bodmer's image is 100 vs 64
#define SHADOW      20
#define MAX_SPICLOCK 8000000  //Saleae must be <= 8MHz. 9341 will work up to 42MHz

/*
 ILI9341 240x320 2.2" LCD pinout (header at the top, from left):
 #1 MISO  -> NC
 #2 LED   -> 3.3V
 #3 SCK   -> SCL/D13/PA5
 #4 SDI   -> MOSI/D11/PA7
 #5 DC    -> any digital
 #6 RESET -> any digital
 #7 CS    -> any digital
 #8 GND   -> GND
 #9 VCC   -> 3.3V
*/

#include "ball_tft_lib.h"  //defines constructor(), TFT_BEGIN(), DRAWIMAGE()

#if !defined(F_CPU)
#define F_CPU SystemCoreClock
#endif

//#define RGBto565(r, g, b) (((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3))
#define RGBto565(r, g, b) lcd.color565(r, g, b)  //all color TFT libraries have this
#define BLACK    0x0000
#define WHITE    0xFFFF
#define RED      0xF800

#include "ball.h"
#include "bigball.h"

uint16_t palette[16];
uint16_t line[SCR_WD];
uint16_t bgCol    = RGBto565(160, 160, 160);
uint16_t bgColS   = RGBto565(90, 90, 90);
uint16_t lineCol  = RGBto565(150, 40, 150);
uint16_t lineColS = RGBto565(80, 10, 80);

#define LINE_YS  20
#define LINE_XS1 30
#define LINE_XS2 6

#if USE_BIGBALL
//#define BALL_HT     BALLHEIGHT   //Bodmer's image is 100
//#define BALL_WD     (BALLWIDTH - 24) //Bodmer's is 136 w shadow. 136x100
#define BALL_ADS    bigball      //array in PROGMEM
#define FIRST_COLOR 2            //Bodmer's image starts at 2
#define OVRLAP      4            //draw extra pixels to rub out prev
#else
//#define BALL_HT     64           //Pawel's image is 64
//#define BALL_WD     BALL_HT      //64x64 bitmap
//#undef BALLWIDTH                 //cos already defined in bigball.h
//#define BALLWIDTH   BALL_HT      //64x64 bitmap
#define BALL_ADS    ball         //array in PROGMEM
#define FIRST_COLOR 1            //palette starts at 2
#define OVRLAP      2            //draw extra pixels to rub out prev
#endif
#define BALL_SWD SCR_WD          //240
#define BALL_SHT (SCR_HT - 60)   //260

#define GRID 20

// AVR stats 16MHz Uno with SPI ILI9341:
// with shadow        - 36ms
// without shadow     - 31ms
uint16_t BALL_WD, BALL_HT, BALL_WIDTH;

void drawBall(int x, int y)
{
    static int oldx; //C9341 41ms. D9341
    int wid = BALL_WD + SHADOW + OVRLAP;
    int i, j;
    int vlines = (SCR_WD / GRID) - 2; //10
    uint16_t color;
    if (oldx > x) oldx = x;
    if (oldx + wid >= SCR_WD) wid = SCR_WD - oldx;
    for (j = 0; j <= BALL_HT; j++) {
        uint8_t v, *img = (uint8_t*)BALL_ADS + 16 * 2 + 6 + j * BALL_WIDTH / 2 + BALL_WD / 2;
        int yy = y + j;
        //redraw the whole background line in the buffer.   when we only need oldx
        if (yy && ((yy - LINE_YS) % GRID) == 0) { //draw horiz graticule
            for (i = oldx; i < oldx + wid; i++) line[i] = lineCol;
            for (i = oldx; i < LINE_XS1; i++) line[i] = bgCol;  //tidy up LHS
            for (i = SCR_WD - LINE_XS1 + 1; i < oldx + wid && i < SCR_WD; i++) line[i] = bgCol;  //tidy up RHS
        } else { //draw vert graticule
            for (i = oldx; i < oldx + wid; i++) line[i] = bgCol;
            if (yy > LINE_YS) for (i = LINE_XS1; i <= SCR_WD - LINE_XS1; i += GRID) line[i] = lineCol;
        }
        if (j < BALL_HT) {
            uint8_t inball = 0, vv = 0;
            img -= BALL_WD / 2;
            for (i = 0; i < BALL_WD; ) {
                uint8_t half;
                v = pgm_read_byte(img++);
                vv = v >> 4;
                for (half = 0; half < 2; i++, half++, vv = v & 0x0F) {
                    if (vv < FIRST_COLOR) {
                        if (inball == 0) continue;  //left of ball
                        else goto shit_out;         //right of ball
                    }
                    inball = 1;                     //inside ball
                    line[x + i] = palette[vv];
                }
            }
shit_out:
            vv = SHADOW;
            if (vv > SCR_WD - x - i) vv = SCR_WD - x - i;
            for ( ; inball && vv-- > 0; i++) {
                color = line[x + i];
                if (color == bgCol) color = bgColS;
                else if (color == lineCol) color = lineColS;
                line[x + i] = color;
            }
        }
        DRAWIMAGE(oldx, yy, wid, 1, line + oldx);  //C9341=36ms. B9341=57ms. A9341=61ms.
    }
    oldx = x;
}

uint16_t LH(const uint8_t *ads)
{
    uint16_t ret = pgm_read_byte(ads + 1);
    return (ret << 8) + pgm_read_byte((ads));
}

void setup()
{
    TFT_BEGIN();
    lcd.setRotation(ORIENTATION);
    lcd.invertDisplay(SCR_INV);
    lcd.fillScreen(bgCol);
    int i, o;
    int vlines = (SCR_WD / GRID) - 2; //10
    int hlines = (SCR_HT / GRID) - 3; //13
    //uint16_t *pal = (uint16_t*)BALL_ADS + 3;
    //for (i = 0; i < 16; i++) palette[i] = pgm_read_word(&pal[i]); //assumes LITTLE-ENDIAN
    BALL_HT = LH(BALL_ADS + 0);
    BALL_WIDTH = LH(BALL_ADS + 2);
    BALL_WD = (BALL_HT == 64) ? 64 : BALL_WIDTH - 24;
    for (i = 0; i < 16; i++) palette[i] = LH( BALL_ADS + 6 + i * 2); //assumes LITTLE-ENDIAN
    for (i = 0; i < vlines; i++) lcd.drawFastVLine(LINE_XS1 + i * GRID, LINE_YS, (hlines - 1) * GRID, lineCol);
    for (i = 0; i < hlines; i++) lcd.drawFastHLine(LINE_XS1, LINE_YS + i * GRID, SCR_WD - LINE_XS1 * 2, lineCol);
    lcd.drawFastHLine(LINE_XS2, SCR_HT - LINE_YS, SCR_WD - LINE_XS2 * 2, lineCol);
    int dy = SCR_HT - LINE_YS - (LINE_YS + GRID * (hlines - 1));
    int dx = LINE_XS1 - LINE_XS2;
    o = 2 * 7 * dx / dy;
    lcd.drawFastHLine(LINE_XS2 + o, SCR_HT - LINE_YS - 7 * 2, SCR_WD - LINE_XS2 * 2 - o * 2, lineCol);
    o = 2 * (7 + 6) * dx / dy;
    lcd.drawFastHLine(LINE_XS2 + o, SCR_HT - LINE_YS - (7 + 6) * 2, SCR_WD - LINE_XS2 * 2 - o * 2, lineCol);
    o = 2 * (7 + 6 + 4) * dx / dy;
    lcd.drawFastHLine(LINE_XS2 + o, SCR_HT - LINE_YS - (7 + 6 + 4) * 2, SCR_WD - LINE_XS2 * 2 - o * 2, lineCol);
    for (i = 0; i < vlines; i++) lcd.drawLine(LINE_XS1 + i * GRID, LINE_YS + GRID * (hlines - 1), LINE_XS2 + i * (SCR_WD - LINE_XS2 * 2) / (vlines - 1), SCR_HT - LINE_YS, lineCol);
    //delay(10000);
    lcd.setTextColor(BLACK, WHITE);
    lcd.setTextSize(2);
    lcd.setCursor(1, lcd.height() - 16);
    lcd.print("  ms <");
    if (USE_TFT_LIB == 0) lcd.print("00000");
    else lcd.print((uint32_t)USE_TFT_LIB, 16);
    lcd.print("> ");
    lcd.print((uint32_t)F_CPU/1000000, 10);
    lcd.print("MHz");
}

int anim = 0, animd = 2;
int x = 0, y = 0;
int xd = 4, yd = 1;
unsigned long ms;

void loop()
{
    ms = micros();
    for (int i = 0; i < 14; i++) {
        palette[i + FIRST_COLOR] = ((i + anim) % 14) < 7 ? WHITE : RED;
    }
    drawBall(x, y);
    anim += animd;
    if (anim < 0) anim += 14;
    x += xd;
    y += yd;
    if (x < 0) {
        x = 0;
        xd = -xd;
        animd = -animd;
    }
    if (x >= BALL_SWD - BALL_WD) {
        x = BALL_SWD - BALL_WD;
        xd = -xd;
        animd = -animd;
    }
    if (y < 0) {
        y = 0;
        yd = -yd;
    }
    if (y >= BALL_SHT - BALL_HT) {
        y = BALL_SHT - BALL_HT;
        yd = -yd;
    }
    ms = micros() - ms;
    ms = (ms +  500) / 1000;
    lcd.setCursor(1, lcd.height() - 16);
    if (ms < 10) lcd.print(' ');
    lcd.print(ms);
    //lcd.print("ms  ");

}