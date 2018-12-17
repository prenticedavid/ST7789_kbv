/*
Serial.println("1. select correct defines for your board wiring");
Serial.println("2. select correct interface type");
Serial.println("3. select appropriate report function");
Serial.println("");
Serial.println("certain registers read first byte e.g. 0x00-0x0F, 0xDA-0xDC");
Serial.println("other multi-argument registers read first byte as dummy");
Serial.println("");
*/

#if defined(ESP32)
#define TFT_MOSI 23
#define TFT_MISO 19
#define TFT_SCK  18
#define TFT_SS   5
#define TFT_DC   13     //DC=7 for HX8347
#define TFT_RESET 12   //Backlight on HX8347
#elif defined(ESP8266)
#define TFT_MOSI D11
#define TFT_MISO D12
#define TFT_SCK  D13
#define TFT_SS   D10
#define TFT_DC   D9     //DC=7 for HX8347
#define TFT_RESET D8   //Backlight on HX8347
#else                   //suits Uno, Nucleo, ...
#define TFT_MOSI 11
#define TFT_MISO 12
#define TFT_SCK  13
#define TFT_SS   10
#define TFT_DC  (9)     //DC=7 for HX8347
#define TFT_RESET (8)   //Backlight on HX8347
#endif

#define NINEBITS  (1<<0)
#define SDA_BIDIR (1<<1)
#define HAS_MISO  (1<<2)
#define IS_9341   (1<<3)
#define IS_9481   (1<<4)
#define IS_9486   (1<<5)
const char *chip = "controller";
char interface = SDA_BIDIR;    //ST7789 bidirectional SDA + DC
//char interface = NINEBITS | SDA_BIDIR;  //3-wire SPI bidirectional SDA
//char interface = NINEBITS | HAS_MISO;  //4-wire SDA + SDO
//char interface = HAS_MISO | IS_9341;  //ILI9341 SPI
//char interface = HAS_MISO | IS_9481;  //ILI9481 SPI
//char interface = HAS_MISO | IS_9486;  //ILI9486 SPI
//char interface = HAS_MISO;  //regular SPI
//char interface = NINEBITS | SDA_BIDIR | IS_9481;  //ILI9481 3-wire SPI bidirectional pin

uint32_t ID = 0;

void setup() {
    // put your setup code here, to run once:
    Serial.begin(9600);

    Serial.println("");
    Serial.println("1. select correct defines for your board wiring");
    Serial.println("2. select correct interface type");
    Serial.println("3. select appropriate report function");
    Serial.println("");
    Serial.println("certain registers read first byte e.g. 0x00-0x0F, 0xDA-0xDC");
    Serial.println("other multi-argument registers read first byte as dummy");
    Serial.println("");

    if (interface & HAS_MISO)
        Serial.println("Read registers 8-bit SPI with MISO (+ DC)");
    else if (interface & NINEBITS)
        Serial.println("Bi-directional Read registers 9-bit SPI");
    else
        Serial.println("Bi-directional Read registers SDA (+ DC)");
    digitalWrite(TFT_SS, HIGH);
    //    digitalWrite(TFT_SCK, HIGH);
    pinMode(TFT_SS, OUTPUT);
    pinMode(TFT_SCK, OUTPUT);
    pinMode(TFT_MOSI, OUTPUT);
    pinMode(TFT_MISO, INPUT);
    pinMode(TFT_DC, OUTPUT);
    pinMode(TFT_RESET, OUTPUT);
    reset_pulse();
    soft_reset();
    ID = readwrite8(0x04, 24, 1);
    ID = readwrite8(0x04, 24, 1);  //ST7789 needs twice
    Serial.print("read reg(4):  ID = 0x");
    Serial.println(ID, HEX);
    if (ID == 0x858552uL) chip = "ST7789V";
    if (ID == 0x7C89F0uL) chip = "ST7735S";
    if (ID == 0x548066uL) chip = "ILI9163C";
    if ((ID & 0xFF0000) == 0x5C0000) chip = "ST7735";
    if (interface & SDA_BIDIR) {
        uint8_t sda_en = 0x80;
        if (interface & IS_9481) writeblock(0xc6, &sda_en, 1);
    }
    read_7735(chip);
    //    read_7735_9481(chip);
    //    read_regs("diagnose any controller");
    //    read_61509("R61509 / ILI9326");
    //    read_9327("ILI9327");
    //    read_9338("ILI9302 / ILI9329 / ILI9338 / ILI9341");
    //    if (interface & NINEBITS) find_sekret();
    //    read_9481("ILI9481 / HX8357 / R61581");
    //    read_9486("ILI9486 / 1LI9488");
}

void reset_pulse(void)
{
    digitalWrite(TFT_RESET, HIGH);
    digitalWrite(TFT_RESET, LOW);   //Hardware Reset
    delay(10);
    digitalWrite(TFT_RESET, HIGH);
    delay(10);
}

void soft_reset(void)
{
    writeblock(0x01, NULL, 0);   //software reset
    delay(10);
}

void read_7735(const char *title)
{
    Serial.println(title);
    Serial.println("data sheet specific calls");
    show7735(0x04, 3, 1);  //RDDID
    show7735(0x09, 4, 1);  //RDDSTATUS
    show7735(0x0A, 1, 0);
    show7735(0x0B, 1, 0);   //RDDMADCTL
    show7735(0x0C, 1, 0);   //RDDCOLMOD
    show7735(0x0D, 1, 0);
    show7735(0x0E, 1, 0);
    show7735(0x0F, 1, 0);
    show7735(0xDA, 1, 0);   //RDID1
    show7735(0xDB, 1, 0);   //RDID2
    show7735(0xDC, 1, 0);   //RDID3
}

void read_7735_9481(const char *title)
{
    Serial.println(title);
    Serial.println("data sheet specific calls");
    show7735(0x0A, 1, 0);
    show7735(0x0B, 1, 0);   //RDDMADCTL
    show7735(0x0C, 1, 0);   //RDDCOLMOD
    show7735(0x0D, 1, 0);
    show7735(0x0E, 1, 0);
    show7735(0x0F, 1, 0);
    show7735(0xA1, 4, 1);  //ILI9481 DDB_Start
    show7735(0xBF, 4, 1);  //ILI9481 ID
}

uint32_t readwrite8(uint8_t cmd, uint8_t bits, uint8_t dummy)
{
    digitalWrite(TFT_SS, LOW);
    write9(cmd, 0);
    uint32_t ret = read8(bits, dummy);
    digitalWrite(TFT_SS, HIGH);
    return ret;
}

void write9(uint8_t val, uint8_t dc)
{
    pinMode(TFT_MOSI, OUTPUT);
    digitalWrite(TFT_DC, dc);
    if (interface & NINEBITS) {
        digitalWrite(TFT_MOSI, dc);
        delay(1);
        digitalWrite(TFT_SCK, HIGH);
        delay(1);
        digitalWrite(TFT_SCK, LOW);
    }
    for (int i = 0; i < 8; i++) {   //send command
        digitalWrite(TFT_MOSI, (val & 0x80) != 0);
        delay(1);
        digitalWrite(TFT_SCK, HIGH);
        delay(1);
        digitalWrite(TFT_SCK, LOW);
        val <<= 1;
    }
}

void writeblock(uint8_t cmd, uint8_t *block, int8_t N)
{
    uint8_t val = cmd;
    digitalWrite(TFT_SS, LOW);
    write9(cmd, 0);
    while (N-- > 0) write9(*block++, 1);
    digitalWrite(TFT_SS, HIGH);
}

uint32_t read8(uint8_t bits, uint8_t dummy)
{
    uint32_t ret = 0;
    uint8_t SDAPIN = (interface & HAS_MISO) ? TFT_MISO : TFT_MOSI;
    pinMode(SDAPIN, INPUT_PULLUP);
    digitalWrite(TFT_DC, HIGH);
    for (int i = 0; i < dummy; i++) {  //any dummy clocks
        digitalWrite(TFT_SCK, HIGH);
        delay(1);
        digitalWrite(TFT_SCK, LOW);
        delay(1);
    }
    for (int i = 0; i < bits; i++) {  // read results
        ret <<= 1;
        delay(1);
        if (digitalRead(SDAPIN)) ret |= 1;;
        digitalWrite(TFT_SCK, HIGH);
        delay(1);
        digitalWrite(TFT_SCK, LOW);
    }
    return ret;
}

void show7735(uint8_t reg, uint8_t bytes, uint8_t dummy)
{
    uint32_t val, mask = 0x10000000;
    uint8_t bits = bytes * 8;
    for (uint8_t wid = 32; wid > bits; wid -= 4) mask >>= 4;
    //    reset_pulse();    //ST7789V not happy with this
    soft_reset();
    val = readwrite8(reg, bits, dummy);

    Serial.print(chip);
    Serial.print(" reg(0x");
    if (reg < 0x10) Serial.print("0");
    Serial.print(reg , HEX);
    Serial.print(") = 0x");
    while (val < mask) Serial.print("0"), mask >>= 4;
    if (val) Serial.println(val, HEX);
    else Serial.println();
}

void printhex(uint8_t val)
{
    if (val < 0x10) Serial.print("0");
    Serial.print(val, HEX);
}

void loop()
{
    // put your main code here, to run repeatedly:
    yield();
}

