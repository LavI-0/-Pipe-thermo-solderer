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

// debug
#define DEBUG
 
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
  cur_temp,
  d20mm,
  d25mm,
  d32mm  
} parametr;
enum DIAMETR  {
  d20,
  d25,
  d32
}diametr;

const uint8_t pipeDiametr[]={20, 25, 32};
uint16_t pipeTemperature[]={270, 300, 320};
uint8_t pipeDelay[]={5, 7, 9};

uint8_t currentPipeDiametr = d20;
int16_t currentTemperature=0;
uint8_t parametrToChange=cur_temp;

bool calibrationStatus=false;
#define FINISHED 0x00
#define START 0x01
#define OVERLOAD_MEM_ERR 0x02

uint16_t maxTemperature=0;
uint16_t minTemperature=0;


//################################ GPIO ###############################################
#define PIN_REDLED 12    //PB4  pin for arduino mini 
#define PIN_GREENLED 7   //PD7 pin for arduino mini 
#define PIN_SYMYSTOR A0  //PC0  pin for arduino mini 
#define PIN_BUZ 2         //PD2   pin for arduino mini 
#define BEEP_ON  digitalWrite(PIN_BUZ, HIGH)
#define BEEP_OFF  digitalWrite(PIN_BUZ, LOW)
#define SYMYSTOR_OFF  digitalWrite(PIN_SYMYSTOR, LOW)
#define SYMYSTOR_ON  digitalWrite(PIN_SYMYSTOR, HIGH)
//################################ DISPLAY ###############################################
#define dispHigh 64
#define dispWidth 128
#define GRAPHWIDTH 120

#define GRAFMODE 1
#define TEXTMODE 2
uint8_t dispMode = TEXTMODE ;
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
void m_s1i5EnterFunc(void);			//"START"); /////// Prototype
// підменю "STOP"
void m_s1i5sEnterFunc(void);			//"STOP"); /////// Prototype
// підменю "Вибір труби"
void m_s2i1EnterFunc(void){			//"20 мм");
  currentPipeDiametr = d20;
  parametrToChange=d20mm;  
}
void m_s2i2EnterFunc(void){	    //"25 мм");
  currentPipeDiametr = d25;
  parametrToChange=d25mm;  
}		
void m_s2i3EnterFunc(void){	    //"32 мм");
  currentPipeDiametr = d32;
  parametrToChange=d32mm;      
}		
// підменю "Налаштування режимів"
// підменю "Налаштування коеф"
void temperatureManage(void); /////// Prototype
void m_s4i2EnterFunc(void){		//"Автопідбір");
  int16_t temp[300];
  uint16_t tempTopIndex = 0;
  uint32_t time=0, time1s=0;
  calibrationStatus=START;
  // PID calculating variables
  uint16_t t1, t2, t3, tau, tdel;
  float A, B, B0632, K, r;
  float kp, ki, kd;
  //
  myPID.SetMode(MANUAL);
  SYMYSTOR_OFF;
  BEEP_OFF;  
  // Setpoint=200;
  
  // check if its stable heater's temperature
  while (maxTemperature-minTemperature>3){
    // while (millis()-time1s<1000) {;}
    temperatureManage();
    delay(1000);
  }
  #ifdef DEBUG
    Serial.println("PWM 50%% srarted");
  #endif
  // start 50% PWM for 3 sec
  time1s=millis();
  for (uint8_t i=0; i<30; i++){ // 30*(50ms+50ms)=3 sec
    time=millis();
    SYMYSTOR_ON;
    while  (millis()-time<50){;}
    time=millis();
    SYMYSTOR_OFF;
    // make temperature measurement each 1 sec   
    if (millis()-time1s>=1000){ 
      temperatureManage();  
      temp[tempTopIndex]=currentTemperature;
      tempTopIndex++;
      time1s=millis();
    }
    //
    while  (millis()-time<50){;}
  }
  //and continue 50% PWM  until temperature stabilizes
  while (temp[tempTopIndex]-temp[tempTopIndex-5]>3){
    time=millis();
    SYMYSTOR_ON;
    while  (millis()-time<50){;}
    time=millis();
    SYMYSTOR_OFF;
    // make temperature measurement each 1 sec   
    if (millis()-time1s>=1000){ 
      temperatureManage();  
      temp[tempTopIndex]=currentTemperature;
      tempTopIndex++;
      if (tempTopIndex==300){
        calibrationStatus=OVERLOAD_MEM_ERR;
        #ifdef DEBUG
          Serial.println("OVERLOAD_MEM_ERR");
        #endif
        return;
      } 
      time1s=millis();
    //
      while  (millis()-time<50){;}    
    }   
  } 
  // calculate PID(https://pages.mtu.edu/~tbco/cm416/cctune.html)
  // 1) Under Manual mode, wait until the process is at steady state.
  // 2) Next, introduce a step change in the input.
  // 3) Based on the output, obtain an approximate first order process with a time constant t delayed by tDEL units from when the input step was introduced.
  // The values of t and tDEL can be obtained by first recording the following time instances:
  // t0	=	time when input step was initiated
  // t2	=	time when half point occurs
  // t3	=	time when 63.2% point occurs  
  // 4) From the measurements based on the step test: t0, t2, t3, A and B, evaluate the following process parameters:
  // t1 = (t2 - ln(2) t3)/(1 - ln(2))
  // tau = t3 - t1
  // tDEL = t1 - t0
  // K = B/A
  // 5) Based on the parameters K, t and tDEL, the following formulas prescribe the controller parameters Kc, tI and tD:
  // r=tdel/tau
  // Kc=(1/K*r)*(4/3+r/4)
  // ti=tdel*((32+6*r)/(13+8*r))
  // td=tdel*(4/(11+2*r))

  A=50;
  B=currentTemperature;
  B0632=(float)B*0.632F;
  for (uint8_t i=0; i<299; i++){
    if (temp[i]<=(int16_t)(B/2))
      if (temp[i+1]>=(int16_t)(B/2)) t2=i;    
    if (temp[i]<=(int16_t)B0632)
      if (temp[i+1]>=(int16_t)B0632) t3=i;
  }

  t1 = ((float)t2 - 0.632F*(float)t3)/(1 - 0.632F);
  tau = t3 - t1;
  tdel = t1;
  K=B/A;
  r=(float)tdel/(float)tau;
  kp=(1.F/(K*r))*(4.F/3.F+r/4.F);
  // Ki=Kp/ti
  ki=kp/(tdel*((32.F+6.F*r)/(13.F+8.F*r)));
  // Kd=Kp*td 
  kd=kp*(tdel*(4.F/(11.F+2*r)));

  myPID.SetTunings(kp, ki, kd);

  #ifdef DEBUG
    Serial.print ("PWM 50%% finished. tempTopIndex=");
    Serial.println (tempTopIndex);
    Serial.print (" kp=");
    Serial.print (kp); 
    Serial.print (" ki=");
    Serial.print (ki);
    Serial.print (" kd=");
    Serial.println (kd);  
  #endif
  //end of calibrations 
  calibrationStatus=FINISHED;
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
      break;   
    case KD: 
      myPID.SetTunings(myPID.GetKp(), myPID.GetKi(), myPID.GetKd()+0.1);// Kd+=0.1;
      break; 
    case KI: 
      myPID.SetTunings(myPID.GetKp(), myPID.GetKi()+0.1, myPID.GetKd()); // Ki+=0.1;
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
MENU_ITEM(m_s1i1,  m_s1i2,    m_s1i5,      NULL_MENU,  m_s2i1,       NULL, 		      NULL,		         "Вибрати трубу");//
MENU_ITEM(m_s1i2,  m_s1i3,    m_s1i1,      NULL_MENU,  m_s3i1,       NULL, 		      NULL,		         "Налаштувати режим"); // 
MENU_ITEM(m_s1i3,  m_s1i4,	  m_s1i2,      NULL_MENU,  m_s4i1,   	   NULL,		    	NULL,		         "Налаштувати PID");
MENU_ITEM(m_s1i4,  m_s1i5,    m_s1i3,      NULL_MENU,  m_s5i1,       NULL,		    	NULL,            "Вивести на дисплей");
MENU_ITEM(m_s1i5,  m_s1i1,    m_s1i4,      NULL_MENU,  m_s1i5s,      NULL,		    	m_s1i5EnterFunc, "-------START-------");
// підменю "STOP"
MENU_ITEM(m_s1i5s, NULL_MENU, NULL_MENU,   NULL_MENU,  NULL_MENU,    NULL,		    	m_s1i5sEnterFunc, "-------STOP-------");
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

void m_s1i5EnterFunc(void){			//"START");
  myPID.SetMode(AUTOMATIC); 
  // dispMode = TEXTMODE;
  Menu_Navigate(&m_s1i5s);
  parametrToChange=cur_temp;
}
void m_s1i5sEnterFunc(void){			//"STOP");
  myPID.SetMode(MANUAL);
  SYMYSTOR_OFF;
  BEEP_OFF;
  parametrToChange=cur_temp;   
  Menu_Navigate(&m_s1i5); 
}

static void Generic_Write(const char* menuText) { // Generic function to write the text of a menu.
  char buf[40]={};  // two bytes for each UTF8 char
  char str_temp[5]={};
  int doubleToInt=0;

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
    
    if (MENU_NEXT != &NULL_MENU) 
	  {
      sprintf_P(buf, PSTR(" %-18S"), ((Menu_Item_t*)MENU_NEXT)->Text);
      u8g2->drawUTF8(0, 14, buf);//C++	
      // u8g2_drawUTF8(&u8g2, 0, 15, buf);//C
    }
    if (calibrationStatus!=START){ // if NOT start auto calibration of PID show usual viewer  
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
          doubleToInt=(int)(myPID.GetKp()*10);
          sprintf(str_temp, "%2d.%1d", doubleToInt/10, doubleToInt%10);
          break;   
        case KD: 
          doubleToInt=(int)(myPID.GetKd()*10);
          sprintf(str_temp, "%2d.%1d", doubleToInt/10, doubleToInt%10);
          break; 
        case KI: 
          doubleToInt=(int)(myPID.GetKi()*10);
          sprintf(str_temp, "%2d.%1d", doubleToInt/10, doubleToInt%10);
          break;
        case cur_temp: 
          sprintf(str_temp, "%03d", currentTemperature); 
          break; 
        case d20mm: 
          sprintf(str_temp, "20");
          break;   
        case d25mm: 
          sprintf(str_temp, "25");
          break; 
        case d32mm: 
          sprintf(str_temp, "32");
          break;              
        default: 
          break;
        }
        u8g2->setFont(u8g2_font_fub42_tn);	// 42 pixel // C++
        // u8g2->setFont(u8g2_font_fub11_tf);	
        // u8g2_setFont(&u8g2, u8g2_font_fub42_tf);	// 42 pixel // C
        u8g2->drawStr(10,63, str_temp);	// write something to the internal memory// C++
        // u8g2_drawStr(&u8g2, 10,63, str_temp);	// write something to the internal memory// C
      }
    }
    else {  // if user start auto calibration of PID  
      sprintf(buf, "      УВАГА!      ");
      u8g2->drawUTF8(0, 25, buf);    
      sprintf(buf, "Oчікуйте завершення");
      u8g2->drawUTF8(0, 34, buf);
      sprintf(buf, "  Це може зайняти  ");
      u8g2->drawUTF8(0,43, buf);
      sprintf(buf, "      до 10 хв     ");
      u8g2->drawUTF8(0,52, buf);   
      sprintf(buf, "Не робіть нічого!!!");
      u8g2->drawUTF8(0, 61, buf);
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
//#########################################################################################
void temperatureManage (void){
  static unsigned char endDataPoint=0; // last save temperature into array data[]
  
  currentTemperature = thermocouple.readCelsius(); 
  maxTemperature=currentTemperature;
  minTemperature=currentTemperature;  
  if (endDataPoint>=GRAPHWIDTH){ // shift all array one step left
    for (uint8_t i=0; i<GRAPHWIDTH-1; i++){
      if (data[i]>maxTemperature) maxTemperature=data[i]; // find maximum temperature
      if (data[i]<minTemperature) minTemperature=data[i]; // find minimum temperature
      data[i]=data[i+1];
    }
    endDataPoint=GRAPHWIDTH-1;
  }
  data[endDataPoint]=currentTemperature;
  endDataPoint++;
}
void graph (uint16_t* arrDot120, uint16_t maxTemperInInterval, uint8_t xStart, uint8_t yStart, uint8_t graphHeight){//120 dots in arrey// arrDot60 - arrey of 60 numbers; gradsInOneDot - scale of Y-axis; xStart - x coordinate of left bottom graph; yStart - y coordinate of left bottom graph;
  uint8_t scaleY=1;
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
  BEEP_ON;
  //################################ keyboard ###############################################
  keyInit();
  //################################ serial ###############################################  
  #ifdef DEBUG
    Serial.begin(9600);
    Serial.println("Setup start");    
  #endif
//################################ MAX6675 ###############################################
  // pinMode(PIN_MAX6675_VCC, OUTPUT); 
  // pinMode(PIN_MAX6675_GND, OUTPUT); 
  // digitalWrite(gndPin, LOW);  
  // digitalWrite(vccPin, HIGH);
//################################ ss1306 ###############################################    
// if (u8g2.begin()) Serial.println("Display ok!"); //C++
// else Serial.println("Display fail!"); //C++

  if (init_screen()){
    #ifdef DEBUG
      Serial.println("Display ok!");     
    #endif    
  }
  else {
    #ifdef DEBUG
      Serial.println("Display fail!");    
    #endif
    while (1) {BEEP_ON;}  
  }  
 //################################ PID ###############################################  
  //initialize the variables we're linked to
  myPID.SetOutputLimits(1, 50);
  myPID.SetSampleTime(1);
  Input = thermocouple.readCelsius();
  Setpoint = 400;
  //turn the PID on
  // myPID.SetMode(AUTOMATIC); 
//################################ MIcroMenu ###############################################
  Menu_SetGenericWriteCallback(Generic_Write);
  Menu_Navigate(&m_s1i5); 
  #ifdef DEBUG
    Serial.println("Setup finished");
  #endif

  digitalWrite(PIN_REDLED, HIGH);
  BEEP_OFF;
}

void loop() {
  // char str_temp[40];
  // static uint8_t min;
  // static uint8_t sec;
  static uint32_t previousTime1s=0, previousTime20ms=0, previousTime50ms=0, previousTime100ms=0;
  static uint8_t pwmCount=1;

  if ((millis()-previousTime100ms)>=100){// work cycle
    previousTime100ms=millis();
    static uint16_t count=0;

    if (myPID.GetMode()==AUTOMATIC){ // if work started
      count++;       
      if (count==19U){ // start beep 0.5 s
        BEEP_ON;
        return;
      }
      if (count==25U){ // stop beep 0.5 s
        BEEP_OFF;
        return;
      } 
      if (count==25U+pipeDelay[currentPipeDiametr]*10U){
        BEEP_ON;
        return;           
      }
      if (count==25U+pipeDelay[currentPipeDiametr]*10U+1U){
        BEEP_OFF;
        return;           
      } 
      if (count==25U+pipeDelay[currentPipeDiametr]*10U+2U){
        BEEP_ON;
        return;           
      }
      if (count==25U+pipeDelay[currentPipeDiametr]*10U+3U){
        BEEP_OFF;
        return;           
      } 
      if (count==25U+pipeDelay[currentPipeDiametr]*10U+4U){
        BEEP_ON;
        return;           
      }
      if (count==25U+pipeDelay[currentPipeDiametr]*10U+5U){
        BEEP_OFF;
        count=0; // stop working cycle
        return;           
      } 
    }                 
  }

  if ((millis()-previousTime50ms)>=50){// hadle push button
    previousTime50ms=millis();
    if (keyScan()){
      #ifdef DEBUG
        Serial.print("Key pushed = ");    
        Serial.println( key, BIN); 
      #endif
    }
  }

  if ((millis()-previousTime20ms)>=20){// hadle PWM symystor
    previousTime20ms=millis();
////////////////// PWM ////////////////////////
    if (myPID.GetMode()==AUTOMATIC)   {
      if (pwmCount<=Output){
        digitalWrite(PIN_SYMYSTOR, HIGH);
        #ifdef DEBUG
          Serial.println("H");
        #endif
      } 
      else{
        digitalWrite(PIN_SYMYSTOR, LOW);
        #ifdef DEBUG
          Serial.println("L");
        #endif
      } 
      pwmCount++; 
      if (pwmCount==51) pwmCount=0;    
    } 
///////////////////////////////////////////////
  }

  if ((millis()-previousTime1s)>1000){
    previousTime1s=millis();

    // min=millis()/60000;
    // sec=(millis()/1000)%60;
    // sprintf(str_temp, "%02d:%02d", min, sec);
    // Serial.println(str_temp);
    // put your main code here, to run repeatedly:
    temperatureManage();
    Input = currentTemperature;
    myPID.Compute(); 

    Generic_Write(Menu_GetCurrentMenu()->Text);

    #ifdef DEBUG
      Serial.print("currentTemperature=");
      Serial.println(currentTemperature);   
      Serial.print("Output=");
      Serial.println(Output);
    #endif 
  }
}
