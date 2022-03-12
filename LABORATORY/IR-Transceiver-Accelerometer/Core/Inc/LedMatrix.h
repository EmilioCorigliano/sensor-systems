/*
 * LedMatrix.h
 *
 *  Created on: Dec 17, 2021
 *      Author: emili
 */

#ifndef INC_LEDMATRIX_H_
#define INC_LEDMATRIX_H_

int8_t iCol_LedMatrix = 0;
uint8_t toShow[5][2];	// letter to print

// codification of letters
uint8_t letter_0[5] = { 0b0111110, 0b1000111, 0b1001001, 0b1110001, 0b0111110 };
uint8_t letter_1[5] = { 0b0000000, 0b0000000, 0b0000000, 0b0000000, 0b1111111 };
uint8_t letter_2[5] = { 0b0100001, 0b1010001, 0b1001001, 0b1000101, 0b0100011 };
uint8_t letter_3[5] = { 0b0110110, 0b1001001, 0b1001001, 0b1001001, 0b1000001 };
uint8_t letter_4[5] = { 0b0001000, 0b1111111, 0b0001000, 0b0001000, 0b1111000 };
uint8_t letter_5[5] = { 0b1001110, 0b1010001, 0b1010001, 0b1010001, 0b1110001 };
uint8_t letter_6[5] = { 0b1000110, 0b1001001, 0b1001001, 0b1001001, 0b0111110 };
uint8_t letter_7[5] = { 0b1110000, 0b1001100, 0b1000011, 0b1000000, 0b1000000 };
uint8_t letter_8[5] = { 0b0110110, 0b1001001, 0b1001001, 0b1001001, 0b0110110 };
uint8_t letter_9[5] = { 0b1111111, 0b1001000, 0b1001000, 0b1001000, 0b0110000 };
uint8_t letter_A[5] = { 0b0011111, 0b0100100, 0b1000100, 0b0100100, 0b0011111 };
uint8_t letter_B[5] = { 0b0110110, 0b1001001, 0b1001001, 0b1001001, 0b1111111 };
uint8_t letter_C[5] = { 0b1000001, 0b1000001, 0b1000001, 0b1000001, 0b1111111 };
uint8_t letter_D[5] = { 0b0111110, 0b1000001, 0b1000001, 0b1000001, 0b1111111 };
uint8_t letter_E[5] = { 0b1000001, 0b1000001, 0b1001001, 0b1001001, 0b1111111 };
uint8_t letter_F[5] = { 0b1000000, 0b1000000, 0b1001000, 0b1001000, 0b1111111 };

/**
 * @brief converts a letter in the "easy to draw" format to the format accepted by the led matrix
 * @param x the letter in the easy to draw format
 * @param pos the of the letter to insert in the toShow variable
 */
static void encodeLetterToShow(char x) {
	uint8_t *letter;
	switch (x) {
	case '0':
		letter = letter_0;
		break;
	case '1':
		letter = letter_1;
		break;
	case '2':
		letter = letter_2;
		break;
	case '3':
		letter = letter_3;
		break;
	case '4':
		letter = letter_4;
		break;
	case '5':
		letter = letter_5;
		break;
	case '6':
		letter = letter_6;
		break;
	case '7':
		letter = letter_7;
		break;
	case '8':
		letter = letter_8;
		break;
	case '9':
		letter = letter_9;
		break;
	case 'A':
		letter = letter_A;
		break;
	case 'B':
		letter = letter_B;
		break;
	case 'C':
		letter = letter_C;
		break;
	case 'D':
		letter = letter_D;
		break;
	case 'E':
		letter = letter_E;
		break;
	case 'F':
		letter = letter_F;
		break;
	default:
		return;
	}

	for (int i = 0; i < 5; i++) {
		toShow[i][0] = letter[i];
		toShow[i][1] = (uint8_t) pow(2, i);
	}
}

#endif /* INC_LEDMATRIX_H_ */
