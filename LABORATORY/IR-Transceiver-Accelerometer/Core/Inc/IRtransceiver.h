/*
 * IRtransceiver.h
 *
 *  Created on: 20 dic 2021
 *      Author: emili
 */

#ifndef INC_IRTRANSCEIVER_H_
#define INC_IRTRANSCEIVER_H_


//// TRANSCEIVER
void sendString(char *str);
void sendByte(uint8_t byte);

//// TRANSMITTER PART
int iBit = 9; // starting in "don't send anything" mode
uint8_t byteToSend;
uint8_t sending = 0;

//// RECEIVER PART
char data;

/**
 * @brief Tx: calls the sendByte function for each char of the string
 */
void sendString(char *str) {
	for (int i = 0; i < strlen(str); i++) {
		sendByte(str[i]);
	}
}

/**
 * @brief Tx: waits until the previous byte is sent;
 * then sets the new byte to send, resets the index to read the bit and
 * sets the flag "sending" to true
 */
void sendByte(uint8_t byte) {
	// wait to finish the sending of the previous byte (in callback of TIM11)
	while (sending)
		;

	// set next byte to send and "start" the sending
	byteToSend = byte;
	iBit = -1;
	sending = 1;
}

#endif /* INC_IRTRANSCEIVER_H_ */
