#include <Arduino.h>
#include <max6675.h>
#include <U8g2lib.h>
#include <PID_v1.h>
#include "keyboard.h"
#include "screen.h"
#include "globals.h"
#include "MicroMenu.h"

#ifdef U8X8_HAVE_HW_SPI
  #include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
  #include <Wire.h>
#endif

enum PARAMETR {
  d20Temp,
  d25Temp,
  d32Temp,
  d20Time,
  d25Time,
  d32Time,
  KP,
  KI,
  KD,
  cur_temp
} parametr;
enum DIAMETR  {
  d20,
  d25,
  d32
}diametr;

const uint8_t pipeDiametr[]={20, 25, 32};
uint16_t pipeTemperature[]={270, 300, 320};
uint8_t pipeDelay[]={5, 7, 8};

uint8_t currentPipeDiametr = d20;
int16_t currentTemperature=0;
uint8_t parametrToChange;

uint16_t maxTemperature=0;
//################################ GPIO ###############################################
#define PIN_REDLED 12    //PB4  pin for arduino mini 
#define PIN_GREENLED 7   //PD7 pin for arduino mini 
#define PIN_SYMYSTOR A0  //PC0  pin for arduino mini 
#define PIN_BUZ 2         //PD2   pin for arduino mini 
//################################ DISPLAY ###############################################
#define dispHigh 64
#define dispWidth 128
#define GRAPHWIDTH 120

#define GRAFMODE 1
#define TEXTMODE 2
uint8_t dispMode = GRAFMODE; //TEXTMODE ;
uint16_t data[GRAPHWIDTH];
//Prototypes
void graph (uint16_t* arrDot120, uint16_t maxTemperInInterval, uint8_t xStart, uint8_t yStart, uint8_t graphHeight); //120 dots in arrey// arrDot60 - arrey of 60 numbers; gradsInOneDot - scale of Y-axis; xStart - x coordinate of left bottom graph; yStart - y coordinate of left bottom graph;
//################################ PID ###############################################
//Define Variables we'll be connecting to
double Setpoint, Input, Output;
//Specify the links and initial tuning parameters
#define DEFAULT_KP 2
#define DEFAULT_KD 1
#define DEFAULT_KI 5
PID myPID(&Input, &Output, &Setpoint, DEFAULT_KP, DEFAULT_KI, DEFAULT_KD, DIRECT);
//################################ MIcroMenu ###############################################
// підменю "Вибір труби"
void m_s2i1EnterFunc(void){			//"20 мм");
  currentPipeDiametr = pipeDiametr[d20];
}
void m_s2i2EnterFunc(void){	    //"25 мм");
  currentPipeDiametr = pipeDiametr[d25];
}		
void m_s2i3EnterFunc(void){	    //"32 мм");
  currentPipeDiametr = pipeDiametr[d32];  
}		
// підменю "Налаштування режимів"
// підменю "Налаштування коеф"
void m_s4i2EnterFunc(void){		//"Автопідбір");
  Setpoint=200;
  
}
// підменю "Вивести на дисплей"
void m_s5i1EnterFunc(void){			//"Стандартно");
  dispMode = TEXTMODE;
}
void m_s5i2EnterFunc(void){			//"З графіком");
  dispMode = GRAFMODE;
  parametrToChange=cur_temp;
}
// підпідменю  "Для труби 20 мм"
void m_s6i1EnterFunc(void){			//"Температура");
  parametrToChange=d20Temp;
}
void m_s6i2EnterFunc(void){			//"Тривалість нагріву");
  parametrToChange=d20Time;
}
// підпідменю  "Для труби 25 мм"
void m_s7i1EnterFunc(void){			//"Температура");
    parametrToChange=d25Temp;
}
void m_s7i2EnterFunc(void){			//"Тривалість нагріву");
  parametrToChange=d25Time;
}
// підпідменю  "Для труби 23 мм"
void m_s8i1EnterFunc(void){			//"Температура");
  parametrToChange=d32Temp;
}
void m_s8i2EnterFunc(void){			//"Тривалість нагріву");
  parametrToChange=d32Time;
}
// підпідменю  "Ввести вручну"
void m_s9i1EnterFunc(void){			//"Kp");
  parametrToChange=KP;
}
void m_s9i2EnterFunc(void){			//"Ki");
  parametrToChange=KI;
}
void m_s9i3EnterFunc(void){			//"Kd");
  parametrToChange=KD;
}
void m_s10i1EnterFunc(void){			//"Збільшити");
  switch (parametrToChange){
    case d20Temp: 
      pipeTemperature[d20]++;
      break;
    case d20Time: 
      pipeDelay[d20]++;
      break;
    case d25Temp: 
      pipeTemperature[d25]++;
      break;
    case d25Time: 
      pipeDelay[d25]++;
      break;
    case d32Temp: 
      pipeTemperature[d32]++;
      break;
    case d32Time: 
      pipeDelay[d32]++;
      break;   
    case KP: 
      myPID.SetTunings(myPID.GetKp()+0.1, myPID.GetKi(), myPID.GetKd());// Kp+=0.1;
      Serial.print("myPID.GetKp()=");
      Serial.println(myPID.GetKp());
      break;   
    case KD: 
      myPID.SetTunings(myPID.GetKp(), myPID.GetKi(), myPID.GetKd()+0.1);// Kd+=0.1;
      Serial.print("myPID.GetKd()=");
      Serial.println(myPID.GetKd());
      break; 
    case KI: 
      myPID.SetTunings(myPID.GetKp(), myPID.GetKi()+0.1, myPID.GetKd()); // Ki+=0.1;
      Serial.print("myPID.GetKi()=");
      Serial.println(myPID.GetKi());
    default: 
      break;
  }
}
void m_s10i2EnterFunc(void){			//"Зменшити");
  switch (parametrToChange){
    case d20Temp: 
      pipeTemperature[d20]--;
      break;
    case d20Time: 
      pipeDelay[d20]--;
      break;
    case d25Temp: 
      pipeTemperature[d25]--;
      break;
    case d25Time: 
      pipeDelay[d25]--;
      break;
    case d32Temp: 
      pipeTemperature[d32]--;
      break;
    case d32Time: 
      pipeDelay[d32]--;
      break;   
    case KP: 
      myPID.SetTunings(myPID.GetKp()-0.1, myPID.GetKi(), myPID.GetKd());// Kp-=0.1;
      break;   
    case KD: 
      myPID.SetTunings(myPID.GetKp(), myPID.GetKi(), myPID.GetKd()-0.1);// Kd-=0.1;
      break; 
    case KI: 
      myPID.SetTunings(myPID.GetKp(), myPID.GetKi()-0.1, myPID.GetKd()); // Ki-=0.1;
    default: 
      break;
  }
}

// const struct Menu_Item PROGMEM m_s1i1 = {&m_s1i2, &NULL_MENU, &NULL_MENU, &m_s2i1, __null, __null, const char Text[]}
//                 NEXT,      PREVIOUS     PARENT,     CHILD		     SELECTFUNC			ENTERFUNC		      TEXT
MENU_ITEM(m_s1i1,  m_s1i2,    NULL_MENU,   NULL_MENU,   m_s2i1,       NULL, 		    NULL,		         "Вибрати трубу");//
MENU_ITEM(m_s1i2,  m_s1i3,    m_s1i1,      NULL_MENU,   m_s3i1,       NULL, 		    NULL,		         "Налаштувати режим"); // 
MENU_ITEM(m_s1i3,  m_s1i4,	  m_s1i2,      NULL_MENU,   m_s4i1,   	  NULL,		    	NULL,		         "Налаштувати коеф");
MENU_ITEM(m_s1i4,  NULL_MENU, m_s1i3,      NULL_MENU,   m_s5i1,       NULL,		    	NULL,            "Вивести на дисплей");
// підменю "Вибір труби"
MENU_ITEM(m_s2i1,  m_s2i2,    m_s2i3,      m_s1i1,     NULL_MENU,     NULL, 		    m_s2i1EnterFunc,  "20 мм");
MENU_ITEM(m_s2i2,  m_s2i3,    m_s2i1,      m_s1i1,     NULL_MENU,     NULL, 		    m_s2i2EnterFunc,  "25 мм");
MENU_ITEM(m_s2i3,  m_s2i1,    m_s2i2,      m_s1i1,     NULL_MENU,     NULL, 		    m_s2i3EnterFunc,  "32 мм");
// підменю "Налаштування режимів"
MENU_ITEM(m_s3i1,  m_s3i2,    m_s3i3,      m_s1i2,     m_s6i1,        NULL, 		    NULL,             "Для труби 20 мм");
MENU_ITEM(m_s3i2,  m_s3i3,	  m_s3i1,      m_s1i2,     m_s7i1,        NULL, 		    NULL,             "Для труби 25 мм");
MENU_ITEM(m_s3i3,  m_s3i1,	  m_s3i2,      m_s1i2,     m_s8i1,        NULL, 		    NULL,             "Для труби 32 мм");
// підменю "Налаштування коеф"
MENU_ITEM(m_s4i1,  m_s4i2,    m_s4i2,      m_s1i3,     m_s9i1,        NULL,   	    NULL,	            "Ввести вручну");
MENU_ITEM(m_s4i2,  m_s4i1,	  m_s4i1,      m_s1i3,     NULL_MENU,     NULL,         m_s4i2EnterFunc,	"Автопідбір");
// підменю "Вивести на дисплей"
MENU_ITEM(m_s5i1,  m_s5i2,    m_s5i2,      m_s1i4,     NULL_MENU,     NULL,	        m_s5i1EnterFunc,	"Стандартно");
MENU_ITEM(m_s5i2,  m_s5i1,	  m_s5i1,      m_s1i4,     NULL_MENU,     NULL,	        m_s5i2EnterFunc,	"З графіком");
// підпідменю  "Для труби 20 мм"
MENU_ITEM(m_s6i1,  m_s6i2,    m_s6i2,      m_s3i1,     m_s10i1,       NULL, 		    m_s6i1EnterFunc,  "Температура");
MENU_ITEM(m_s6i2,  m_s6i1,	  m_s6i1,      m_s3i1,     m_s10i1,       NULL, 		    m_s6i2EnterFunc,  "Тривалість нагріву");
// підпідменю  "Для труби 25 мм"
MENU_ITEM(m_s7i1,  m_s7i2,    m_s7i2,      m_s3i2,     m_s10i1,       NULL, 		    m_s7i1EnterFunc,  "Температура");
MENU_ITEM(m_s7i2,  m_s7i1,	  m_s7i1,      m_s3i2,     m_s10i1,       NULL, 		    m_s7i2EnterFunc,  "Тривалість нагріву");
// підпідменю  "Для труби 23 мм"
MENU_ITEM(m_s8i1,  m_s8i2,    m_s8i2,      m_s3i3,     m_s10i1,       NULL, 		    m_s8i1EnterFunc,  "Температура");
MENU_ITEM(m_s8i2,  m_s8i1,	  m_s8i1,      m_s3i3,     m_s10i1,       NULL, 		    m_s8i2EnterFunc,  "Тривалість нагріву");
// підпідменю  "Ввести вручну"
MENU_ITEM(m_s9i1,  m_s9i2,    m_s9i3,      m_s4i1,     m_s10i1,       NULL, 		    m_s9i1EnterFunc,  "Kp");
MENU_ITEM(m_s9i2,  m_s9i3,	  m_s9i1,      m_s4i1,     m_s10i1,       NULL, 		    m_s9i2EnterFunc,  "Ki");
MENU_ITEM(m_s9i3,  m_s9i1,	  m_s9i2,      m_s4i1,     m_s10i1,       NULL, 		    m_s9i3EnterFunc,  "Kd");
// підменю редагвання чисел для у всіх підменю підменю налаштування
MENU_ITEM(m_s10i1, m_s10i2,   m_s10i2,     m_s1i2,     NULL_MENU,     NULL, 		    m_s10i1EnterFunc, "Збільшити");
MENU_ITEM(m_s10i2, m_s10i1,   m_s10i1,     m_s1i2,     NULL_MENU,     NULL, 		    m_s10i2EnterFunc, "Зменшити");


static void Generic_Write(const char* menuText) { // Generic function to write the text of a menu.
  char buf[40]={}; 
  char str_temp[5]={};
  Serial.println("Generic_Write()"); 

  u8g2->firstPage(); //C++
  // u8g2_firstPage(&u8g2); //C
  do
  {
    // u8g2->setFont(u8g2_font_haxrcorp4089_t_cyrillic);	// 7 pixel //C++
    u8g2->setFont(u8g2_font_6x12_t_cyrillic);	
    // u8g2->setFont(u8g2_font_5x8_t_cyrillic);
    // u8g2_setFont(&u8g2, u8g2_font_haxrcorp4089_t_cyrillic);	// 7 pixel //C

  // menu for 2 string with ">" in first one
    sprintf_P(buf, PSTR(">%-18S"), (char*)menuText);
    u8g2->drawUTF8(0, 7, buf); //C++
    // u8g2_drawUTF8(&u8g2, 0, 7, buf); //C
    
    // if ((void*)tempMenu != (void*)&NULL_ENTRY) 
    if (MENU_NEXT != &NULL_MENU) 
	  {
      sprintf_P(buf, PSTR(" %-18S"), ((Menu_Item_t*)MENU_NEXT)->Text);
      u8g2->drawUTF8(0, 15, buf);//C++	
      // u8g2_drawUTF8(&u8g2, 0, 15, buf);//C
    }
    
    if (dispMode==GRAFMODE){ // show ghaph
      graph (data, maxTemperature, 5, dispHigh-1, 45);
    }
    else if (dispMode==TEXTMODE){ // show diametr and temperature
      switch (parametrToChange){
      case d20Temp: 
        sprintf(str_temp, "%02d", pipeTemperature[d20]);
        break;
      case d20Time: 
        sprintf(str_temp, "%02d", pipeDelay[d20]);
        break;
      case d25Temp: 
        sprintf(str_temp, "%02d", pipeTemperature[d25]);
        break;
      case d25Time: 
        sprintf(str_temp, "%02d", pipeDelay[d25]);
        break;
      case d32Temp: 
        sprintf(str_temp, "%02d", pipeTemperature[d32]);
        break;
      case d32Time: 
        sprintf(str_temp, "%02d", pipeDelay[d32]);
        break;   
      case KP: 
        sprintf(str_temp, "%4.1f", (float)myPID.GetKp());
        break;   
      case KD: 
        sprintf(str_temp, "%4.1f", (float)myPID.GetKd());
        break; 
      case KI: 
        sprintf(str_temp, "%4.1f", (double)myPID.GetKi());
        break;
      case cur_temp: 
        sprintf(str_temp, "%03d", currentTemperature); 
        break;       
      default: 
        break;
      }
      // u8g2->setFont(u8g2_font_fub42_tn);	// 42 pixel // C++
      u8g2->setFont(u8g2_font_fub11_tf);	
      // u8g2_setFont(&u8g2, u8g2_font_fub42_tf);	// 42 pixel // C
      u8g2->drawStr(10,63, str_temp);	// write something to the internal memory// C++
      // u8g2_drawStr(&u8g2, 10,63, str_temp);	// write something to the internal memory// C
    } 
  } while (u8g2->nextPage());// C++
  // } while (u8g2_nextPage(&u8g2));// C
}
//################################ MAX6675 ###############################################
#define PIN_MAX6675_SO A3
#define PIN_MAX6675_CS A2
#define PIN_MAX6675_SCK A1
MAX6675 thermocouple(PIN_MAX6675_SCK, PIN_MAX6675_CS, PIN_MAX6675_SO);
// make a cute degree symbol
// uint8_t degree[8]  = {140,146,146,140,128,128,128,128};
#define MAXTEMPERATURE 50

void graph (uint16_t* arrDot120, uint16_t maxTemperInInterval, uint8_t xStart, uint8_t yStart, uint8_t graphHeight){//120 dots in arrey// arrDot60 - arrey of 60 numbers; gradsInOneDot - scale of Y-axis; xStart - x coordinate of left bottom graph; yStart - y coordinate of left bottom graph;
  uint8_t scaleY=1;
  // uint16_t temp16=MAXTEMPERATURE; 
  char str_temp[3]={};

  if ((maxTemperInInterval/graphHeight)<1) scaleY=1;
    else if ((maxTemperInInterval/graphHeight)<3) scaleY=2;
      else if ((maxTemperInInterval/graphHeight)<6) scaleY=5;
        else if ((maxTemperInInterval/graphHeight)<11) scaleY=10;
          else if ((maxTemperInInterval/graphHeight)<21) scaleY=20;

  if (xStart<2) xStart=2;
  if (xStart>8) xStart=8;
  if (yStart<graphHeight) yStart=graphHeight;
  if (yStart>dispHigh) yStart=dispHigh-1;

  u8g2->drawHLine(xStart, yStart, dispWidth-8); // C++
  u8g2->drawVLine(xStart, yStart-graphHeight, graphHeight); //C++
  // u8g2_drawHLine(&u8g2, xStart, yStart, dispWidth-8); // C
  // u8g2_drawVLine(&u8g2, xStart, yStart-graphHeight, graphHeight); //C 
  sprintf(str_temp, "x%d", scaleY);
  u8g2->setFont(u8g2_font_pcsenior_8f);	// 6 pixel height //C++
  // u8g2_setFont(&u8g2, u8g2_font_pcsenior_8f); //C 
  u8g2->drawStr(xStart-2, yStart-graphHeight+6, str_temp);//C++
  // u8g2_drawStr(&u8g2, xStart-2, yStart-graphHeight+6, str_temp);//C

  for (int i=yStart; i>yStart-graphHeight+7; i-=5){
    u8g2->drawPixel(xStart-1, i);//C++
    u8g2->drawPixel(xStart-2, i);//C++
    // u8g2_drawPixel(&u8g2, xStart-1, i);//C
    // u8g2_drawPixel(&u8g2, xStart-2, i);//C 
  }

  for (int i=0; i<120; i++){
    u8g2->drawPixel(xStart+i, yStart-arrDot120[i]/scaleY); //C++
    // u8g2_drawPixel(&u8g2, xStart+i, yStart-arrDot120[i]/scaleY); //C
  }
  // Serial.println("Graph builded");
}
void setup() {
//################################ GPIO ###############################################
  pinMode(PIN_REDLED, OUTPUT);
  pinMode(PIN_GREENLED, OUTPUT);
  pinMode(PIN_BUZ, OUTPUT);
  pinMode(PIN_SYMYSTOR, OUTPUT);  
  digitalWrite(PIN_SYMYSTOR, LOW); 
  digitalWrite(PIN_REDLED, LOW);   
  digitalWrite(PIN_GREENLED, LOW);
  digitalWrite(PIN_BUZ, HIGH);
  //################################ keyboard ###############################################
  keyInit();
  //################################ serial ###############################################  
 Serial.begin(9600);
 Serial.println("Setup start");
//################################ MAX6675 ###############################################
  // pinMode(PIN_MAX6675_VCC, OUTPUT); 
  // pinMode(PIN_MAX6675_GND, OUTPUT); 
  // digitalWrite(gndPin, LOW);  
  // digitalWrite(vccPin, HIGH);
//################################ ss1306 ###############################################    
// if (u8g2.begin()) Serial.println("Display ok!"); //C++
// else Serial.println("Display fail!"); //C++

  if (init_screen()) 
    Serial.println("Display ok!"); 
  else 
    Serial.println("Display fail!"); 
//################################ PID ###############################################  
  //initialize the variables we're linked to
  myPID.SetOutputLimits(1, 50);
  Input = thermocouple.readCelsius();
  Setpoint = 400;
  //turn the PID on
  myPID.SetMode(AUTOMATIC); 
//################################ MIcroMenu ###############################################
  Menu_SetGenericWriteCallback(Generic_Write);
  Menu_Navigate(&m_s1i1); 

  Serial.println("Setup finished");
  digitalWrite(PIN_REDLED, HIGH);
  digitalWrite(PIN_BUZ, LOW);
}

void loop() {
  char str_temp[40];//str_temp[6];
  static uint8_t min;
  static uint8_t sec;
  static uint32_t previousTime1s=0, previousTime20ms=0, previousTime50ms=0;
  static unsigned char endDataPoint=0; // last save temperature into array data[]

  static uint8_t pwmCount=1;

  if ((millis()-previousTime50ms)>=50){// hadle push button
    previousTime50ms=millis();
    if (keyScan())
{      Serial.print("Key pushed = ");    
      Serial.println( key, BIN); }
  }

if ((millis()-previousTime20ms)>=20){// hadle PWM symystor
    previousTime20ms=millis();
////////////////// PWM ////////////////////////    
    if (pwmCount<=Output) digitalWrite(PIN_SYMYSTOR, HIGH);
    else digitalWrite(PIN_SYMYSTOR, LOW);
    pwmCount++; 
    if (pwmCount==51) pwmCount=0;
///////////////////////////////////////////////
  }

  if ((millis()-previousTime1s)>1000){
    previousTime1s=millis();

    min=millis()/60000;
    sec=(millis()/1000)%60;
    sprintf(str_temp, "%02d:%02d", min, sec);
    Serial.println(str_temp);
    // put your main code here, to run repeatedly:
    currentTemperature = thermocouple.readCelsius(); 

    maxTemperature=currentTemperature;
    if (endDataPoint>=GRAPHWIDTH){ // shift all array one step left
      for (uint8_t i=0; i<GRAPHWIDTH-1; i++){
        if (data[i]>maxTemperature) maxTemperature=data[i];
        data[i]=data[i+1];
      }
      endDataPoint=GRAPHWIDTH-1;
    }
    data[endDataPoint]=currentTemperature;

    endDataPoint++;

    Input = currentTemperature;
    myPID.Compute(); 

    Generic_Write(Menu_GetCurrentMenu()->Text);
 
    Serial.print("Temperature=");
    Serial.println(currentTemperature);  
  }
  // delay(1000);

}