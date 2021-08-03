#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

extern U8G2_SSD1306_128X64_NONAME_1_HW_I2C *u8g2; //note use of pointer, extern to prevent multiple definitions error

bool init_screen();