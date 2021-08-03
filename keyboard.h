#ifndef _KEYBOARD_H_
#define _KEYBOARD_H_

#include <Arduino.h>
#include "MicroMenu.h"
#include "globals.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SW1		3				// клавіатура 
#define SW2		2
#define SW3		1
#define SW4		0
#define SW_PORT 	PORTB
#define SW_DDR		DDRB
#define SW_PIN		PINB

#define KEY_UP				1
#define KEY_DOWN			2
// #define KEY_RIGHT			4
#define KEY_LEFT			3
#define KEY_OK				4
#define	key_msk 	0b00001111 		// маска встановлення натиснотуї кнопки	
#define	key_msk_new 0b00010000		// маска визначення чи було нове натискання

extern u08 key;		// тут знаходиться одне із чисел (0x01, 0x02, 0x04, 0x08) що відповідає натиснутій кнопці, 0xff -нічого не натиснуто (опрацьовано)

void keyInit(void);
bool keyScan(void); // if any key was pushed - return 1, else return 0.
u08 getKeyCode(u08 keyPush);
u08 menuKey(u08 par);



#ifdef __cplusplus
}
#endif

#endif