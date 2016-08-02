#include <DHT.h>
#include <VarSpeedServo.h>
#include <TaskScheduler.h>
#include <DirectIO.h>

Scheduler runner;

//-------------------------------------------
// Pin usage:
//-------------------------------------------
//
// 0 (RX)			-> Serial
// 1 (TX)			-> Serial
// 2 (INT0)			-> DHT sensor
// 3 (INT1)			-> Motion sensor 
// 4				-> Camera enable  
// 5 				-> Device 1
// 6				-> Device 2
// 7				-> Device 3
// 8 				-> Device 4
// 9				-> Horizontal servo
// 10 (SS)			-> Vertical servo
// 11 (MOSI)		-> Device 5
// 12 (MISO)		-> Device 6
// 13 (SCK)			-> Blinking led
// 14 (A0)			-> Co2 sensor
// 15 (A1)			-> Gas sensor
// 16 (A2)			-> Light sensor
// 17 (A3)			-> Device 7
// 18 (A4, SDA)		-> Device 8
// 19 (A5, SCL)		-> 
//
//-------------------------------------------


//------------------------
// Blink Task section
//------------------------


Output<13> ledPin;

void blink();
Task blinkTask(500, TASK_FOREVER, &blink);

void blink() {
	ledPin = !ledPin;
}


//------------------------
// Serial task section
//------------------------

#define HOR_SERVO_PIN 9
#define VER_SERVO_PIN 10

OutputPin devices[] = { OutputPin(5), OutputPin(6), OutputPin(7), OutputPin(8), OutputPin(11), OutputPin(12), OutputPin(17), OutputPin(19) };

VarSpeedServo horServo;
VarSpeedServo verServo;

void serialCommand();
Task serialCommandTask(1000, TASK_FOREVER, &serialCommand);

#define COMMAND_SIZE 3
#define SERVO_SPEED 30

// TODO ALSO THERE CAN BE SIREN COMMAND!
void serialCommand() {
	// COMMANDS can be turn camera and turn smth on/off
	// On/off are bits and hence one byte can contain 8 switches
	byte buffer[COMMAND_SIZE] = { 0 };
	if (Serial.available() > 0) {
		// read command
		Serial.readBytes(buffer, COMMAND_SIZE);

		// flush buffer (ignore all the rest bytes)
		while (Serial.available())
			Serial.read();

		auto hor_pos = constrain(buffer[0], 0, 180);
		auto ver_pos = constrain(buffer[1], 0, 180);
		horServo.write(hor_pos, SERVO_SPEED, true);
		verServo.write(ver_pos, SERVO_SPEED, true);

		byte deviceStates = buffer[2];
		byte mask = 1;
		for (int i = 0; i < 8; i++) {
			mask <<= 1;
			devices[i] = deviceStates & mask;
		}
	}
}

//------------------------
// Sensor read stat task
//------------------------

void reader();
Task readSensors(1200, TASK_FOREVER, &reader); // add &reader, &runner, true to avoid adding tasks and enabling them manually

#define CO2PIN A0 // MQ7 - co2 (products of combustion)
#define GASPIN A1 // MQ2 - gas and smoke (flamable and combustible gases)
#define LIGHTPIN A2 

#define MOTION_PIN 3
#define CAMERA_ALLOWED_PIN 4
#define DHT_PIN 2

Input<MOTION_PIN> motionSensor;
Input<CAMERA_ALLOWED_PIN> cameraAllowed;

DHT humiditySensor(DHT_PIN, DHT11);

volatile int co2, gas, motion, camAllowed, light;
volatile float humidity, temperature;

void reader() {
	co2 = analogRead(CO2PIN);
	gas = analogRead(GASPIN);
	light = analogRead(LIGHTPIN);
	motion = motionSensor.read();
	camAllowed = cameraAllowed.read();

	humidity = humiditySensor.readHumidity();
	temperature = humiditySensor.readTemperature();

	// todo replace strings with bytes (first map from 1024 to 256, then send as bytes)
	Serial.print(co2);
	Serial.print(";");
	Serial.print(gas);
	Serial.print(";");
	Serial.print(light);
	Serial.print(";");
	Serial.print(motion);
	Serial.print(";");
	Serial.print(cameraAllowed);
  Serial.print(";");
  Serial.print(humidity);
  Serial.print(";");
  Serial.print(temperature);
  Serial.println("");
	// todo send humidity, temperature 
}


void setup() {
	// Initializing
	runner.init();

	// Common init
	Serial.begin(9600);

	// Blink task init
	runner.addTask(blinkTask);

	// Serial task init
	horServo.attach(HOR_SERVO_PIN);
	verServo.attach(VER_SERVO_PIN);
	runner.addTask(serialCommandTask);

	// Sensor status task
	humiditySensor.begin();
	runner.addTask(readSensors);

	// Starting
	runner.enableAll();
}

void loop() {
	runner.execute();
}
