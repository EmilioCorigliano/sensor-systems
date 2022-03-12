/*
 * Accelerometer.h
 *
 *  Created on: Dec 17, 2021
 *      Author: emili
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

#define accel_addr (0b0101000<<1) 	// address of the LIS2DE (shift by one for the R/W bit)
#define AUTOINCREMENT (0b1<<7)		// bit to set high if we want the auto increment (to set with the register address)

/**
 * @brief enumeration that contains the association between register and their address
 */
typedef enum {
	WHO_AM_I = 0x0F,
	TEMP_CFG_REG = 0x1F,
	CTRL_REG1 = 0x20,
	CTRL_REG4 = 0x23,
	OUT_X = 0x29,
	OUT_Y = 0x2B,
	OUT_Z = 0x2D,
	OUT_TEMP_L = 0x0C,
	OUT_TEMP_H = 0x0D
} Reg;

/**
 * @brief data structure that contains a sample from the accelerometer
 */
typedef struct {
	float acc_x;
	float acc_y;
	float acc_z;
} AccelData;

float convertToAcc(int8_t data);

AccelData accel_rcv;
AccelData accelData; 	// structure that contains the last sample
char sample_str[64];	// string that contains the message to send over UART
uint8_t accel_buf[8]; // variable that contains the data sent in DMA mode by the sensor
uint8_t accelDataToBeSent = 0;
uint8_t accel_isReceived = 0;


/**
 * @brief converts a byte into the acceleration in g
 * @param data the byte read from the accelerometer
 * @return the float signed value of the acceleration in g
 */
float convertToAcc(int8_t value) {
	// multiplying the read data by the FSR and dividing by 2^bytes=256
	return (float) value * 4 / 256.0;
}

#endif /* INC_ACCELEROMETER_H_ */
