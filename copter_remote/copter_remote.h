#ifndef copter_remote_H_
#define copter_remote_H_

#include "Arduino.h"
#include "SPI.h"
#include "Mirf/Mirf.h"
#include "Mirf/nRF24L01.h"
#include "Mirf/MirfHardwareSpiDriver.h"

#ifdef __cplusplus
extern "C" {
#endif
void loop();
void setup();
#ifdef __cplusplus
} // extern "C"
#endif

/* Radio definitions */
#define RADIO_CHANNEL	108
#define RADIO_PAYLOAD	sizeof(radio_packet)
#define COPTER_ADDR 	"coptr"
#define BASE_ADDR		"basss"

typedef struct radio_packet {
	char motor; // motor 1,2,3,4 or all
	int speed; // range 700-2000
	short int ack; // Set to 1 to acknowledge a command.
} radio_packet;

//Do not add code below this line
#endif /* copter_remote_H_ */
