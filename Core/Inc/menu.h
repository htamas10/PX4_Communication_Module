#ifndef MENU_H
#define MENU_H

typedef enum {
	MAIN_MENU, SUB_MENU,
} MenuState;

volatile typedef enum {
	UART_CONFIG, ADD_WEIGHT, MANUAL_SET, OPTIONS,
} MainMenuItems;

void Update_Main_Menu();
void Display_UART_Config();
int Display_Add_Weight();
void Display_Manual_Set();
void Display_Options();
void Update_Sub_Menu();

#endif
