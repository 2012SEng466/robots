#include "hover_station.h"

#define RADIO_PIN						5
#define MOTOR_PIN						7
#define ECHO_PIN						11
#define TRIG_PIN						12
#define ONBOARD_LED_PIN					13
#define INDICATOR_LED					6

// Polling delay for loop, depending on whether desired height is met
#define POLLING_DELAY_STABLE			1500
#define POLLING_DELAY_FAST				750

#define MOTOR_MIN_OPERATING_SPEED		85
#define MOTOR_MAX_SPEED					175
// motor increments as slowly as possible. This gives us more precision.
#define MOTOR_INCREMENT					1

#define DISTANCE_LOW					1
#define DISTANCE_HIGH					20
// Add this if you want the ball to be considered at desired height when it is off
#define DISTANCE_ERROR_MARGIN			0

// this must be volatile because it's used in an ISR (radio_rxhandler).
volatile uint8_t rxflag = 0;
radiopacket_t packet;
uint8_t station_addr[5] = { 0x66, 0x66, 0x66, 0x66, 0x66 };

Servo motor;
int motor_current_speed = 0; // global var that persistently tracks the motor's speed

// Calibrations for the ball balancing
int desired_distance_current = 8; // Persistently tracks desired distance. Start it somewhere.

// returns distance in inches of sensed object.
int doSonar()
{
	// Sonar setup stuff
	digitalWrite(TRIG_PIN, HIGH);                  // Send a 10uS high to trigger ranging
	delayMicroseconds(10);
	digitalWrite(TRIG_PIN, LOW);

	// Now ping for objects
	int distance = pulseIn(ECHO_PIN, HIGH);        // Read in times pulse
	distance = distance / 148;                     // Calculate distance from time of pulse

	// We aren't worried about sensing very high, so if it's above a certain
	// maximum, for our intents and purposes, ignore it and say there's nothing there
	if (distance >= DISTANCE_HIGH) {
		distance = 0;
	}
	Serial.print("distance: ");
	Serial.println(distance);
	return distance;
}

void motorSetSpeed(int speed)
{
	if (speed > MOTOR_MAX_SPEED)
		speed = MOTOR_MAX_SPEED;
	else if (speed < MOTOR_MIN_OPERATING_SPEED)
		speed = MOTOR_MIN_OPERATING_SPEED;

	motor.write(speed);
	motor_current_speed = speed;

	Serial.print("current motor speed = ");
	Serial.println(motor_current_speed);
}

void motorSpeedUp()
{
	int increment;
	// set increment to 3 depending on speed
	increment = motor_current_speed <= 90 ? 3 : MOTOR_INCREMENT;
	motorSetSpeed(motor_current_speed + increment);
}

void motorSlowDown()
{
	motorSetSpeed(motor_current_speed - MOTOR_INCREMENT);
}

void motorStop()
{
	motor.write(0);
}

// !!! Very important! Motor has to start at 0 and reach desired speed gradually.
// It will NOT work if you go directly to speed 60 or 100, etc.
void motorStartAt(int start_speed)
{
	int i;
	for (i=0; i < start_speed; i+=5) {
		motor.write(i);
		Serial.println(i);
		delay(500);
	}
}


/* Gets and returns the button number from the base station.
 * If it receives a wrong button, or something goes wrong, returns -1.
 */
int radio_check()
{
	char output[128];

	if (rxflag)	{
		if (Radio_Receive(&packet) != RADIO_RX_MORE_PACKETS) {
			// if there are no more packets on the radio, clear the receive flag;
			// otherwise, we want to handle the next packet on the next loop iteration.
			rxflag = 0;
		}

		// This station is only expecting to receive MESSAGE packets.
		if (packet.type != MESSAGE) {
			snprintf(output, 128, "Error: wrong packet type (type %d).\n\r", packet.type);
			Serial.print(output);
			return -1;
		}

		// Set the transmit address to the one specified in the message packet.
		Radio_Set_Tx_Addr(packet.payload.message.address);

		// Print out the message, along with the message ID and sender address.
		snprintf(output, 128, "Message ID %d from 0x%.2X%.2X%.2X%.2X%.2X: '%s'\n\r",
				packet.payload.message.messageid,
				packet.payload.message.address[0],
				packet.payload.message.address[1],
				packet.payload.message.address[2],
				packet.payload.message.address[3],
				packet.payload.message.address[4],
				packet.payload.message.messagecontent);
		Serial.print(output);

		// Reply to the sender by sending an ACK packet.
		packet.type = ACK;

		if (Radio_Transmit(&packet, RADIO_WAIT_FOR_TX) == RADIO_TX_MAX_RT) {
			snprintf(output, 128, "Could not reply to sender.\n\r");
			Serial.print(output);
		} else {
			snprintf(output, 128, "Replied to sender.\n\r");
			Serial.print(output);
		}
		// delay for a bit to give a visible flash on the LED.
		delay(50);
		digitalWrite(ONBOARD_LED_PIN, LOW);

		return packet.payload.message.messagecontent[0] - '0';
	}

	return -1;
}

/**
 * This function is a hook into the radio's ISR.  It is called whenever the radio generates an RX_DR (received data ready) interrupt.
 */
void radio_rxhandler(uint8_t pipenumber)
{
	// just set a flag and toggle an LED.  The flag is polled in the main function.
	rxflag = 1;
	digitalWrite(ONBOARD_LED_PIN, HIGH);
}

void recv_radio_setup() {
	pinMode(RADIO_PIN, OUTPUT);	// radio's Vcc pin is connected to this digital pin.

	// reset the radio
	digitalWrite(RADIO_PIN, LOW);
	delay(100);
	digitalWrite(RADIO_PIN, HIGH);
	delay(100);

	// initialize the radio, including the SPI module
	Radio_Init();

	// configure the receive settings for radio pipe 0
	Radio_Configure_Rx(RADIO_PIPE_0, station_addr, ENABLE);
	// configure radio transciever settings.
	Radio_Configure(RADIO_2MBPS, RADIO_HIGHEST_POWER);
}

/**
 * Triggers the sonar and performs motor logic to increase or
 * decrease the speed.
 * Returns true if the motor has changed speed or false if stayed
 */
int alterMotor(int desired_distance)
{
	static int zero_distance_count = 0; // tracks number zeros seen

	Serial.print("Desired distance is: ");
	Serial.println(desired_distance);

	// If we sense a distance of "0" 5 times in a row, it was 0.
	// Otherwise, it was probably just a false reading.
	int distance = doSonar();
	if (distance == 0) {
		if (zero_distance_count == 5) {
			motorSetSpeed(MOTOR_MIN_OPERATING_SPEED);
		} else {
			zero_distance_count++;
		}
		return 0;
	} else {
		zero_distance_count = 0;
	}

	if (distance >= (desired_distance - DISTANCE_ERROR_MARGIN) && distance <= (desired_distance + DISTANCE_ERROR_MARGIN)) {
		digitalWrite(INDICATOR_LED, HIGH);
		return 0;
	} else {
		digitalWrite(INDICATOR_LED, LOW);
		if (distance <= desired_distance) {
			Serial.println("Speeding up!");
			motorSpeedUp();
		} else {
			Serial.println("Slowing down!");
			motorSlowDown();
		}
		return 1;
	}
}

void setNewDesiredDistance(int msg)
{
	int tmp;
	int tmp_distance;
	switch (msg) {
		case 4:
			tmp = 1;
			break;
		case 2:
			tmp = -1;
			break;
		default:
			tmp = 0;
			break;
	}
	tmp_distance = desired_distance_current + tmp;
	tmp = (tmp_distance <= DISTANCE_HIGH && tmp_distance >= DISTANCE_LOW)? tmp : 0;
	desired_distance_current = desired_distance_current + tmp;
}

void setup()
{

	// Default Ardiuno setups
	sei(); // for some premade radio setup that we don't know
	init(); // arduino default init stuff

	delay(2000); // give time to open serial connection
	Serial.begin(9600);

	// Motor
	Serial.println("Attaching...");
	delay(1500);
	motor.attach(MOTOR_PIN);
	motorStop();
	delay(1000);
	motorStartAt(MOTOR_MIN_OPERATING_SPEED);

	// LED!!
	pinMode(ONBOARD_LED_PIN, OUTPUT);
	pinMode(INDICATOR_LED, OUTPUT);
	digitalWrite(INDICATOR_LED, LOW);

	// Radio
	recv_radio_setup();

	// Sonar
	pinMode(ECHO_PIN, INPUT);
	pinMode(TRIG_PIN, OUTPUT);

	Serial.println("Hover station initialized.");
}

int main()
{
	int msgInt, loop_delay;

	setup();

	for (;;) {

		Serial.println("---------------------------------");

		// Get Radio Message
		msgInt = radio_check();

		// If message received, update desired height
		if (msgInt != -1) {
			Serial.print("Received ");
			Serial.println(msgInt);
			setNewDesiredDistance(msgInt);
		}

		// Poll slower when desired height reached
		if (alterMotor(desired_distance_current))
			loop_delay = POLLING_DELAY_FAST;
		else
			loop_delay = POLLING_DELAY_STABLE;

		delay(loop_delay);
	}

	return 0;
}
