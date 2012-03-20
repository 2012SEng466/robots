// Do not remove the include below
#include "copter_control.h"

/**
 * Radio Pins:
 * Hardware SPI:
 * MISO -> 12
 * MOSI -> 11
 * SCK -> 13
 * CE -> 8
 * CSN -> 7
 */

int radio_flag = 0;

/**
 * Initializes nRF24L01 Radio
 */
void radio_init(char * addr, int channel, int payload){

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

void radio_irq() {
	radio_flag = 1;
}

/**
 * Checks if a packet has been received.
 * If a packet has been received, data is added to radio_packet pointer
 * and 1 is returned.
 * Otherwise, 0 is returned.
 */
int radio_check_recv(radio_packet * packet) {
	/*
	 * If a packet has been received.
	 */
	if (!Mirf.isSending() && Mirf.dataReady()) {
		/*
		 * Get load the packet into the buffer.
		 */
		Mirf.getData((byte *) packet);
		return 1;
	} else {
		/* No data */
		return 0;
	}
}

/**
 * Sends a radio_packet to receiver at address addr and returns.
 */
void radio_send_nowait(radio_packet * packet, char * addr) {
	Mirf.setTADDR((byte *)addr);
	Mirf.send((byte *)&packet);
}

void setup() {
	Serial.begin(9600);

	radio_init(COPTER_ADDR, RADIO_CHANNEL, RADIO_PAYLOAD);
	attachInterrupt(0,radio_irq,LOW); // Wait for interrupt from radio before checking

	Serial.println("Listening...");
}

void loop() {
	radio_packet packet;

	// Check if data received
	if (!radio_check_recv(&packet))
		return;

	// DEBUG - interrupt should have been set
	if (radio_flag == 1)
		radio_flag = 0;
	else
		Serial.println("Warning: Radio Flag was not set.");

	packet.ack = 1;

	radio_send_nowait(&packet, BASE_ADDR);
	// I think it can only send every 10ms... read the spec
	delay(10); // Quick fix - wait for radio. Later check time since last sent with millis() in send?
}
