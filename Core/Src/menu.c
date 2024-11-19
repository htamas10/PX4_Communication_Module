#include "menu.h"
#include "lcd.h"
#include "main.h"
#include <stdbool.h>
#include <stdio.h>

#define PAYLOAD_EMPTY_WEIGHT 150

MenuState menu_state;
MainMenuItems main_menu_items;

const char *main_items[] = { "UART CONFIG", "ADD WEIGHT", "MANUAL SET",
		"OPTIONS" };

uint16_t weight = PAYLOAD_EMPTY_WEIGHT;

void Update_Main_Menu() {
	int j = 0;
	int displayWidth = 123;

	LCD_Clear();

	for (int i = 0; i < 4; i++) {
		int textWidth = strlen(main_items[i]) * 6;
		LCD_WriteString(main_items[i], (displayWidth - textWidth) / 2, j);
		if (main_menu_items == i) {
			LCD_WriteString("<<", 100, i * 2);
		}
		j += 2;
	}
}

void Display_UART_Config() {

}

int Display_Add_Weight() {
	int y, length;
	char str[20];
	bool weight_changed = false;

	while (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin)) {
		if (!HAL_GPIO_ReadPin(LCD_BTN2_GPIO_Port, LCD_BTN2_Pin)
				&& weight < 500) {
			HAL_Delay(200);
			weight += 50;
			weight_changed = true;
		}

		if (!HAL_GPIO_ReadPin(LCD_BTN1_GPIO_Port, LCD_BTN1_Pin)
				&& weight > PAYLOAD_EMPTY_WEIGHT) {
			HAL_Delay(200);
			weight -= 50;
			weight_changed = true;
		}

		if (weight_changed) {
			LCD_Clear();
			weight_changed = false;
		}

		if (weight >= 100) {
			y = 48;
		}

		else {
			y = 54;
		}

		length = snprintf(NULL, 0, "%d", weight);
		snprintf(str, length + 1, "%d", weight);

		LCD_WriteString("WEIGHT:", (126 - strlen("WEIGHT:") * 6) / 2, 2);
		LCD_WriteString("<", 28, 4);
		LCD_WriteString(">", 90, 4);
		LCD_WriteString(str, y, 4);
		LCD_WriteString("G", 69, 4);
	}
}

void Display_Options() {
	int brightness, length;
	int y;
	char str[20];
	bool brightness_changed = false;

	while (HAL_GPIO_ReadPin(BTN_LEFT_GPIO_Port, BTN_LEFT_Pin)) {
		if (!HAL_GPIO_ReadPin(LCD_BTN2_GPIO_Port, LCD_BTN2_Pin)
				&& TIM2->CCR1 < TIM2->ARR - 1000) {
			HAL_Delay(200);
			TIM2->CCR1 += 1000;
			brightness_changed = true;
		}

		else if (!HAL_GPIO_ReadPin(LCD_BTN1_GPIO_Port, LCD_BTN1_Pin)
				&& TIM2->CCR1 > 500) {
			HAL_Delay(200);
			TIM2->CCR1 -= 1000;
			brightness_changed = true;
		}

		if (brightness_changed) {
			LCD_Clear();
			brightness_changed = false;
		}

		brightness = TIM2->CCR1 / 100;

		if (brightness >= 100) {
			y = 48;
		}

		else {
			y = 54;
		}

		length = snprintf(NULL, 0, "%d", brightness);
		snprintf(str, length + 1, "%d", brightness);

		LCD_WriteString("BRIGHTNESS:", 32, 2);
		LCD_WriteString("<", 32, 4);
		LCD_WriteString(">", 90, 4);
		LCD_WriteString(str, y, 4);
		LCD_WriteString("%", 69, 4);
	}
}

void Display_Manual_Set() {

}

void Update_Sub_Menu() {
	LCD_Clear();

	switch (main_menu_items) {

	case UART_CONFIG:
		Display_UART_Config();
		break;

	case ADD_WEIGHT:
		Display_Add_Weight();
		break;

	case MANUAL_SET:
		Display_Manual_Set();
		break;

	case OPTIONS:
		Display_Options();
		break;
	}

	HAL_Delay(10);
	menu_state = MAIN_MENU;
	Update_Main_Menu();
}
