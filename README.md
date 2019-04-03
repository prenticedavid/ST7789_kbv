# ST7789_kbv
Library for TFT SPI modules :
ST7735   128x160 (and 128x128)
GC9101   128x128
ILI9163  128x128
ST7789   240x240 with no /CS pin
ILI9341  240x320
ILI9481  320x480
ILI9488  320x480

Some ST7735, GC9101, ILI9163 128x128 modules are configured for 128x160.   
Use offset in constructor(ID, w, h, offset, xor) and xor to configure directions.

This ST7789 240x240 Ebay module has no /CS pin and is strictly 3.3V for VCC and GPIO.  #define USE_NO_CS
ILI9481, ILI9488 can only operate in 666 mode.   It is optional for the other controllers.   #define USE_666 1
The ST7789 only starts up in SPI mode3.   Controllers with CS pin can use both mode#0 and mode#3

I have not implemented readGRAM(), readID(), ... yet
Nor have I implemented 9-bit bidirectional driver for ILI9481 yet
