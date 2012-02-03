// Do not remove the include below

#include "arduino/Arduino.h"
#include "radio.h"
#include <avr/interrupt.h>

volatile uint8_t rxflag = 0;

uint8_t station_addr[5] = { 0x66, 0x66, 0x66, 0x66, 0x66 };
uint8_t my_addr[5] = { 0x77, 0x77, 0x77, 0x77, 0x77 };

radiopacket_t packet;

void radio_begin()
{
	pinMode(5, OUTPUT);

	digitalWrite(5, LOW);
	delay(100);
	digitalWrite(5, HIGH);
	delay(100);

	Radio_Init();

	// configure the receive settings for radio pipe 0
	Radio_Configure_Rx(RADIO_PIPE_0, my_addr, ENABLE);
	// configure radio transceiver settings.
	Radio_Configure(RADIO_2MBPS, RADIO_HIGHEST_POWER);

	Radio_Set_Tx_Addr(station_addr);

	delay(1000);
}

void radio_send(char * msg)
{
	packet.type = MESSAGE;
	memcpy(packet.payload.message.address, my_addr, RADIO_ADDRESS_LENGTH);
	packet.payload.message.messageid = 55;
	snprintf((char*)packet.payload.message.messagecontent, sizeof(packet.payload.message.messagecontent), msg);
	Radio_Transmit(&packet, RADIO_WAIT_FOR_TX);

	// wait for retransmit
	while (!rxflag);

	if (Radio_Receive(&packet) != RADIO_RX_MORE_PACKETS)
	{
		// if there are no more packets on the radio, clear the receive flag;
		// otherwise, we want to handle the next packet on the next loop iteration.
		rxflag = 0;
	}

	// DEBUG
	if (packet.type == ACK)
	{
		digitalWrite(3, !digitalRead(3));
	}
}

int main()
{
	int data;

	sei(); // enable global interrupts, may not be required
	init(); // init arduino

	// enable debug LED pin
	pinMode(3, OUTPUT);
	// enable debug serial
	Serial.begin(9600);

	// wait a little bit, seems to help
	delay(2000);

	// DEBUG
	Serial.println("Program begin");
	digitalWrite(3, LOW);

	// initialize radio settings
	radio_begin();

	for (;;) {
		if (Serial.available())
			data = Serial.read();
		else
			continue;

		data = data - '0';

		switch (data) {
		case 0:
			digitalWrite(3, HIGH);
			radio_send("1");
			break;
		case 1:
			digitalWrite(3, LOW);
			radio_send("2");
			break;
		case 2:
			radio_send("3");
			break;
		case 3:
			radio_send("4");
			break;
		default:
			break;
		}

	}
	return 0;
}

void radio_rxhandler(uint8_t pipe_number)
{
	rxflag = 1;
}
