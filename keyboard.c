#include "keyboard.h"
//=====================KeyBoard==================================================================
u08 key=0xF0;		// тут знаходиться одне із чисел (0x01, 0x02, 0x04, 0x08) що відповідає натиснутій кнопці, 0xff -нічого не натиснуто (опрацьовано)

void keyInit(void){
	SW_DDR&=~((1<<SW1)|(1<<SW2)|(1<<SW3)|(1<<SW4));
	SW_PORT|=(1<<SW1)|(1<<SW2)|(1<<SW3)|(1<<SW4); // Pull-up enable
}

bool keyScan(void) {
	u08 keyBit;
	keyBit=(~SW_PIN)&key_msk;
	if (keyBit==0){
		key=0;
		return 0;
	}
	if ((key&key_msk)!=keyBit) {// якщо попередня кнопка була натиснута інакша
		menuKey(getKeyCode(keyBit));
		key=keyBit; 			// оновити значення перемінної кнопок 
		return 1;	
	}
}
u08 getKeyCode(u08 keyPush) {
	switch(keyPush)	
	{
		case 0x01:	return KEY_DOWN; break;
		case 0x02:	return KEY_UP; break;
		case 0x04:	return KEY_LEFT; break;
		case 0x08:	return KEY_OK; break;
	}	
	return 0;
}
u08 menuKey(u08 par) {			// what to do if any button is on
	switch (par) {
		case KEY_LEFT: {
			Menu_Navigate(MENU_PARENT);
			//key=key&(~key_msk_new); // delete pushed key flag 
			break;
		}
		case KEY_DOWN: {
			Menu_Navigate(MENU_NEXT);
			//key=key&(~key_msk_new); // delete pushed key flag 
			break;
		}	
		case KEY_OK:{ 
			Menu_EnterCurrentItem();
			Menu_Navigate(MENU_CHILD);
			//key=key&(~key_msk_new); // delete pushed key flag 
			break;
		}
		case KEY_UP: { 
			Menu_Navigate(MENU_PREVIOUS);
			//key=key&(~key_msk_new); // delete pushed key flag 
			break;
		}
		default: 
			return 0;
	}
}