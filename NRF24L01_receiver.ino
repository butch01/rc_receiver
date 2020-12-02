

// setup radio
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <ArduinoLog.h>


#define DEBUG_FREE_MEMORY 0

#if DEBUG_FREE_MEMORY == 1
	#include <MemoryFree.h>
#endif

// radio
// ce = yellow
#define CE 7
//#define A0
// csn = green
//#define CSN 8
#define CSN 8
RF24 radio(CE, CSN); // CE, CSN

// setup protocol
#define PACKET_LENGTH 18
unsigned char protocolValuesOld[PACKET_LENGTH];
unsigned long lastPacketTime=0;
unsigned long packetTimeout=2000; // set packet timeout to 1 second

// setup servos
/**
 * enhancedServo is using megaServo. MegaServo is using arduino hw timer1.
 * While using Timer1, we cannot use PWM Ports 9 and 10 with analogWrite. They can still be used as digital ports.
 * see also https://www.pjrc.com/teensy/td_libs_TimerOne.html
 */
#include <EnhancedServo.h>

EnhancedServo steeringServo;
#define STEERING_SERVO_PIN 3
#define PROTOCOL_CHANNEL_STEERING 2
#define STEERING_SERVO_MIN 0
#define STEERING_SERVO_MAX 180


// setup motor
#include "MotorDriverTB6612FNG.h"
#define PROTOCOL_CHANNEL_MOTOR_1 1
// channel 1
#define MOTOR_1_IN_1 10
#define MOTOR_1_IN_2 9
#define MOTOR_1_PWM 6
#define MOTOR_1_STDBY 4

// STETUP LIGHTS
#define ALARM_LIGHT_0_PIN A0
#define ALARM_LIGHT_1_PIN A1
unsigned long alarmTimer0=0;
//unsigned long alarmTimer1=0;
#define ALARM_BLINK_INTERVAL 150
unsigned char alarmBlinkStatus=0;

#define BREAK_LIGHTS_PIN 5
#define REAR_LIGHTS_PIN 5



#define HEADLIGHT_PIN A2


// confiure braking behavior
#define MOTOR_1_ENABLE_BREAK true
#define MOTOR_1_SOFTBRAKE_TRIGGER 10
#define MOTOR_1_HARD_BRAKE_TRIGGER 100
#define MOTOR_1_BRAKE_NEUTRAL_REVERSE_TIME 200
// channel 2
//#define MOTOR_2_IN_1 12
//#define MOTOR_2_IN_2 13
//#define MOTOR_2_PWM 5
//#define MOTOR_2_STDBY  8

MotorDriverTB6612FNG motor;


// stateful protocol vars
bool isPacketProcessed = false;


unsigned long lastMicros=0;
const byte address[6] = "00001";
void setup()
{

	Serial.begin(115200);

	#if DEBUG_FREE_MEMORY == 1
	  Serial.print("freeMemory()=");
	  Serial.println(freeMemory());
	#endif


	// initialize lastPacket array
	for (unsigned char i; i< PACKET_LENGTH;i++)
	{
		protocolValuesOld[i]=0;
	}


	Serial.println(F("Starting receiver!"));

	Log.begin   (LOG_LEVEL_VERBOSE, &Serial);


	Log.notice(F("setting up NRF24L01\n"));

	#if DEBUG_FREE_MEMORY == 1
	  Serial.print("freeMemory()=");
	  Serial.println(freeMemory());
	#endif


	Serial.println(F("a1"));
	radio.begin();
	Log.notice(F("is chip connected %b" CR), radio.isChipConnected());
	Serial.println(F("a2"));
	radio.setDataRate( RF24_250KBPS );
	Serial.println(F("a3"));
	//radio.printDetails();
	//Serial.println(F("a4"));
	radio.setPALevel(RF24_PA_MIN);
	Serial.println(F("a5"));
	radio.openReadingPipe(0, address);
	Serial.println(F("a6"));

	Serial.print("CRC: ");
	Serial.println(radio.getCRCLength());

	radio.setRetries(0, 0);

	radio.setAutoAck(false);

	radio.startListening();
//	delay(1000);

	 Serial.println(F("1"));
	// setup steeringServo
	steeringServo.attach(STEERING_SERVO_PIN);
	steeringServo.setMaxValue(STEERING_SERVO_MAX);
	steeringServo.setMinValue(STEERING_SERVO_MIN);

	Serial.println(F("2"));
	// setup motor
	motor = MotorDriverTB6612FNG(MOTOR_1_IN_1, MOTOR_1_IN_2, MOTOR_1_PWM, MOTOR_1_STDBY);

	motor.configureBreaks(MOTOR_1_ENABLE_BREAK, MOTOR_1_SOFTBRAKE_TRIGGER, MOTOR_1_HARD_BRAKE_TRIGGER, MOTOR_1_BRAKE_NEUTRAL_REVERSE_TIME);
	motor.setBackwardsDirection(DIRECTION_CW);

	Serial.println(F("3"));
	// setup lights
	pinMode (ALARM_LIGHT_0_PIN, OUTPUT);
	pinMode (ALARM_LIGHT_1_PIN, OUTPUT);
	pinMode (HEADLIGHT_PIN, OUTPUT);

	pinMode (REAR_LIGHTS_PIN, OUTPUT);
	pinMode (BREAK_LIGHTS_PIN, OUTPUT);

	// DEBUG enable all lights
	digitalWrite(HEADLIGHT_PIN, true);
	digitalWrite(ALARM_LIGHT_0_PIN, true);
	digitalWrite(ALARM_LIGHT_1_PIN, true);
	digitalWrite(REAR_LIGHTS_PIN, true);
	analogWrite(REAR_LIGHTS_PIN, 255);
	Serial.println(F("setup done"));

}


// blinking alarm lights
void runAlarmLightBlink()
{
	unsigned long currentTime = millis();
	if (currentTime - ALARM_BLINK_INTERVAL > alarmTimer0 )
	{
		if (alarmBlinkStatus == 1)
		{
			digitalWrite(ALARM_LIGHT_0_PIN, true);
			digitalWrite(ALARM_LIGHT_1_PIN, false);
			alarmBlinkStatus=0;
		}
		else
		{
			digitalWrite(ALARM_LIGHT_1_PIN, true);
			digitalWrite(ALARM_LIGHT_0_PIN, false);
			alarmBlinkStatus=1;
		}
		alarmTimer0=currentTime;
	}


}

void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X ", num);
  Serial.print(hexCar);
}


/**
 * this function processes everything what is to do if packet timeout occurs
 */
void doPacketTimeout()
{
	// Log
	Serial.println(F("PACKET-TIMEOUT"));

	// keep steeringServo position. Do not touch it.

	// stop the motor (short brake)
	 motor.shortBreak();
	//motor.stop();
}


void loop()
{

	unsigned long currentMillis = millis();

	// declare here to only use inside this loop iteration
	byte protocolValuesCurrent[PACKET_LENGTH];
	if (radio.available())
	{
		// save packet to
//		Serial.print(F("packet received: "));
		radio.read(&protocolValuesCurrent, PACKET_LENGTH);
//		Serial.print (sizeof (protocolValuesCurrent));
//		for (int i=0; i< sizeof (protocolValuesCurrent); i++)
//		{
//			//printHex(protocolValuesCurrent[i]);
//			//Serial.print(text[i]);
//			Serial.print(protocolValuesCurrent[i]);
//			Serial.print(" ");
//		}
//		Serial.print(F("\n"));

		// we have a new packet -> update lastPacketTime
		lastPacketTime = currentMillis;
		// mark packet as not processd
		isPacketProcessed=false;

	}


	// check if we are in timeout
	if (lastPacketTime  + packetTimeout < currentMillis)
	{
		// we are on timeout
		doPacketTimeout();
	}
	else
	{
		if (!isPacketProcessed)
		{
			// no timeout, process packet as usual


			// update the servo
			//Serial.print(F("set: steering: "));
			//Serial.print(protocolValuesCurrent[PROTOCOL_CHANNEL_STEERING]);
			steeringServo.enhancedWrite(protocolValuesCurrent[PROTOCOL_CHANNEL_STEERING]);

			// Serial.print(F(" motor: "));
			//Serial.println(protocolValuesCurrent[PROTOCOL_CHANNEL_MOTOR_1]);
			motor.movePWMTwoWay(protocolValuesCurrent[PROTOCOL_CHANNEL_MOTOR_1], 0, 255);

//			Log.notice(F("set: steering: %d motor: %d breakLight: %T reverseLight: %T prevDM: %d" CR), protocolValuesCurrent[PROTOCOL_CHANNEL_STEERING], protocolValuesCurrent[PROTOCOL_CHANNEL_MOTOR_1], motor.isBreakingLightsOn(), motor.isBackwardsDrivingLightOn(), motor.getDriveModePrevious() );
			Serial.print(F("S="));
			Serial.print(protocolValuesCurrent[PROTOCOL_CHANNEL_STEERING]);
			Serial.print(F(" M="));
			Serial.print(protocolValuesCurrent[PROTOCOL_CHANNEL_MOTOR_1]);
			Serial.print(F("\n"));

			// update protocolValuesLast with values of protocolValuesCurrent. Maybe we need it later
			for (unsigned char i=0; i< PACKET_LENGTH; i++)
			{
				protocolValuesOld[i] = protocolValuesCurrent[i];
			}

			// mark packet as processed
			isPacketProcessed=true;

			// debug memory
			#if DEBUG_FREE_MEMORY == 1
			  Serial.print("freeMemory()=");
			  Serial.println(freeMemory());
			#endif
		}

	}
	// run alarmLight
	runAlarmLightBlink();

}
