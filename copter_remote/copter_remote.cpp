// Do not remove the include below
#include "copter_remote.h"

/**
 * Pins:
 * Hardware SPI:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 * CE -> 8
 * CSN -> 9
 */

/**
 * Initializes nRF24L01 Radio
 */
void radio_init(char * addr, int channel, int payload) {
	/*
	 * Setup pins / SPI.
	 */
	Mirf.cePin = 8;
	Mirf.csnPin = 9;

	Mirf.spi = &MirfHardwareSpi;
	Mirf.init();

	/*
	 * Configure reciving address.
	 */
	Mirf.setRADDR((byte *)addr);

	/*
	 * Set the payload length to sizeof(unsigned long) the
	 * return type of millis().
	 *
	 * NB: payload on client and server must be the same.
	 */
	Mirf.payload = payload;

	/*
	 * Write channel and payload config then power up receiver.
	 */

	/*
	 * To change channel:
	 *
	 * Mirf.channel = 10;
	 *
	 * NB: Make sure channel is legal in your area.
	 */
	Mirf.channel = channel;

	Mirf.config();
}

/**
 * Sends a radio_packet to receiver at address addr and waits
 * for send to finish.
 */
void radio_send(radio_packet * packet, char * addr) {

	Mirf.setTADDR((byte *)addr);

	Mirf.send((byte *)&packet);

	while(Mirf.isSending()){
	}
	// DEBUG
	Serial.println("Finished sending");
	delay(10);
}

/**
 * Waits to receive data over a given timeout period.
 * Data put into packet structure give via pointer.
 */
int radio_recv(unsigned int timeout, radio_packet * packet) {
	unsigned long time = millis();
	while(!Mirf.dataReady()){
		//Serial.println("Waiting");
		if ( ( millis() - time ) > timeout ) {
			Serial.println("Timeout on response from copter!");
			return 0;
		}
	}

	Mirf.getData((byte *) packet);
	return 1;
}

void setup() {
	Serial.begin(9600);

	// init radio address, channel, payload size
	radio_init(BASE_ADDR, RADIO_CHANNEL, RADIO_PAYLOAD);

	Serial.println("Beginning ... ");
}

void loop() {
	radio_packet packet;
	packet.speed = 700;
	packet.motor = 'a';
	packet.ack = 0;

	radio_send(&packet, COPTER_ADDR);

	// Receive data before 1000ms
	if (!radio_recv(1000, &packet)) {
		Serial.println("Failed to receive response.");
		delay(1000);
		return;
	}

	if (packet.ack == 1) {
		Serial.println("Send and receive success");
	} else {
		Serial.println("No Ack received.");
	}

	delay(1000);
}


