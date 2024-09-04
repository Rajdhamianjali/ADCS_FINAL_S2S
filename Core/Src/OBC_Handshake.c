/*
 * OBC_Handshake.c
 *
 *  Created on: Sep 1, 2024
 *      Author: Anjali
 */

#include "OBC_Handshake.h"

#define ACK_HEAD	(0x53)
#define ACK_TAIL	(0x7E)
#define ACK_LENGTH	(7)

extern uint8_t OBC_HANDSHAKE_FLAG;
uint8_t MainCMDHs[ACK_LENGTH];

void WAIT_FOR_HANDSHAKE() {

	memset(MainCMDHs, '\0', ACK_LENGTH);
	OBC_HANDSHAKE_FLAG = 0;
	if (HAL_UART_Receive(&huart1, MainCMDHs, ACK_LENGTH, 7000) == HAL_OK){
		myprintf("--> Handshake command received from OBC: 0x%x\r\n");
		for (int i = 0; i < (ACK_LENGTH); i++) {
			myprintf("%02x ", MainCMDHs[i]);
		}
		myprintf("\n");

		uint8_t header = 0x00;

		if (MainCMDHs[0] == header) {

			for (int loop1 = 0; loop1 < sizeof(MainCMDHs); loop1++) {
				MainCMDHs[loop1] = MainCMDHs[loop1 + 1];
			}
		}

		if (MainCMDHs[0] == ACK_HEAD && MainCMDHs[5] == ACK_TAIL) {
			myprintf("--> Command Acknowledged successful!\n");
			if (HAL_UART_Transmit(&huart1, MainCMDHs, ACK_LENGTH, 2000)
					== HAL_OK){
				myprintf("--> Handshake ACK, re-transmit to OBC: \n");
				for (int i = 0; i < (ACK_LENGTH); i++) {
					myprintf("%02x ", MainCMDHs[i]);
				}
				myprintf("\n");
				OBC_HANDSHAKE_FLAG = 1;
				memset(MainCMDHs, '\0', ACK_LENGTH);
			}
		} else {
			myprintf("*** Unknown Handshake command received!\n");
			if (HAL_UART_Transmit(&huart1, MainCMDHs, ACK_LENGTH, 2000)
					== HAL_OK) {
				myprintf("--> Unknown Handshake ACK, re-transmit to OBC.\n");
				for (int i = 0; i < (ACK_LENGTH); i++) {
					myprintf("%02x ", MainCMDHs[i]);
				}
				myprintf("\n");
				memset(MainCMDHs, '\0', ACK_LENGTH);
				OBC_HANDSHAKE_FLAG = 0;
				WAIT_FOR_HANDSHAKE();
			}
		}
	} else {
		OBC_HANDSHAKE_FLAG = 0;
		myprintf("*** Handshake Command receive failed, try again!\n");
		memset(MainCMDHs, '\0', ACK_LENGTH);
		WAIT_FOR_HANDSHAKE();
	}
}
