/*
 * Keyboard.h
 *
 *  Created on: Dec 17, 2021
 *      Author: emili
 */

#ifndef INC_KEYBOARD_H_
#define INC_KEYBOARD_H_

#include "IRtransceiver.h"

int8_t iCol_Keyboard = 0;
int8_t pressed[4][4];
char keyboard_mapping[4][4] = { { '0', '1', '2', '3' }, { '4', '5', '6', '7' },
		{ '8', '9', 'A', 'B' }, { 'C', 'D', 'E', 'F' } };

//// KEYBOARD
uint8_t readCol();
void init_Keyboard();
void setCol(uint8_t iCol);
void init_Keyboard();
void scanKeyboard();
void keyboardCallback();

uint8_t readCol() {
	return (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3) << 3)
			+ (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2) << 2)
			+ (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) << 1)
			+ (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_12) << 0);
}

void setCol(uint8_t iCol) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11,
			((iCol == 3) ? GPIO_PIN_SET : GPIO_PIN_RESET));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10,
			((iCol == 2) ? GPIO_PIN_SET : GPIO_PIN_RESET));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9,
			((iCol == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8,
			((iCol == 0) ? GPIO_PIN_SET : GPIO_PIN_RESET));

}

void init_Keyboard() {
	// initialized the pressed matrix
	for (int i = 0; i < 4; i++)
		memset(pressed[i], 0, 4);

	// KEYBOARD
	setCol(iCol_Keyboard);
}

void scanKeyboard(){
	// scans all the keyboard
	for (uint8_t x = 0; x < 4; x++) {
		for (uint8_t y = 0; y < 4; y++) {
			if (pressed[x][y] == 3) {
				pressed[x][y]++;
				sendByte(keyboard_mapping[x][y]);
			}
		}
	}
}

void keyboardCallback(){
	//// KEYBOARD
	uint8_t row = readCol();

	// writing the letter for each button pressed in that column
	for (int iRow = 0; iRow < 4; iRow++) {
		if (!(row & (1 << iRow))) {
			if (pressed[iRow][iCol_Keyboard] < 3) {
				pressed[iRow][iCol_Keyboard]++;
			}
		} else {
			pressed[iRow][iCol_Keyboard] = 0;
		}
	}

	// changing the column
	iCol_Keyboard = (iCol_Keyboard + 1) % 4;
	setCol(iCol_Keyboard);
}

#endif /* INC_KEYBOARD_H_ */
