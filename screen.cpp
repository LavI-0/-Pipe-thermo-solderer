#include "screen.h"
#include "globals.h"

U8G2_SSD1306_128X64_NONAME_1_HW_I2C *u8g2;   // actually define the pointer here just the once.

// original constructor
//################################ U8G2_SSD1306 ###############################################
// #define PIN_WIRE_SDA        (18)
// #define PIN_WIRE_SCL        (19)
//for arduino mini SDA - PC4(A4); SCL - PC5(A5)

//C++
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
// U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ A5, /* data=*/ A4, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display FULL BUFFER
// U8G2_SSD1306_128X64_NONAME_1_SW_I2C u8g2(U8G2_R0, /* clock=*/ A5, /* data=*/ A4, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display PAGE BUFFER
 
  // if (u8g2.begin()) Serial.println("Display ok!"); //C++
  // else Serial.println("Display fail!"); //C++

  // for C
  // u8g2_Setup_ssd1306_i2c_128x64_noname_1(u8g2, rotation, u8x8_byte_sw_i2c, uC specific);// [page buffer, size = 128 bytes]
  // u8g2_Setup_ssd1306_i2c_128x64_noname_2(u8g2, rotation, u8x8_byte_sw_i2c, uC specific);// [page buffer, size = 256 bytes]
  // u8g2_Setup_ssd1306_i2c_128x64_noname_f(u8g2, rotation, u8x8_byte_sw_i2c, uC specific);// [full framebuffer, size = 1024 bytes]
  // u8g2_Setup_ssd1306_i2c_128x64_noname_2(&u8g2, U8G2_R0, u8x8_byte_sw_i2c, u8x8_gpio_and_delay_arduino_mini);  // init u8g2 structure
  // u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,
  // u8g2_SetPowerSave(&u8g2, 0); // wake up display
bool init_screen(){
	// note use of new keyword
  // u8g2 = new U8G2_SSD1306_128X64_NONAME_1_SW_I2C(U8G2_R0, /* clock=*/ A5, /* data=*/ A4, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display PAGE BUFFER
	u8g2 = new U8G2_SSD1306_128X64_NONAME_1_HW_I2C (U8G2_R0, /* reset=*/ U8X8_PIN_NONE); 
  if (u8g2->begin()){// use -> instead of . to access methods now
    u8g2->enableUTF8Print();
    return 1;   
  } 

  else 
    return 0; 
}