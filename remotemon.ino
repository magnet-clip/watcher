#include <VarSpeedServo.h>
#include <TaskScheduler.h>
#include <DirectIO.h>

Scheduler runner;


// Blink Task section
Output<13> ledPin;
void blink();
Task blinkTask(500, TASK_FOREVER, &blink);

void blink() {
	ledPin = !ledPin;
}


// Serial task section
#define HOR_SERVO_PIN 9
#define VER_SERVO_PIN 10

VarSpeedServo horServo;
VarSpeedServo verServo;

void serialCommand();
Task serialCommandTask(100, TASK_FOREVER, &serialCommand);

#define COMMAND_SIZE 4

void serialCommand() {
	byte buffer[COMMAND_SIZE] = { 0 };
	if (Serial.available() > 0) {
		Serial.readBytes(buffer, COMMAND_SIZE);
		auto hor_pos = map(buffer[0], 0, 255, 0, 180);
		auto ver_pos = map(buffer[1], 0, 255, 0, 180);
		horServo.write(hor_pos, 30, true);
		verServo.write(ver_pos, 30, true);


	}
}


// Sensor read stat task
void reader();
Task readSensors(1000, TASK_FOREVER, &reader);

#define CO2PIN A0 // MQ7 - co2 (products of combustion)
#define GASPIN A1 // MQ2 - gas and smoke (flamable and combustible gases)
#define MOTION_PIN 3
Input<3> motionSensor;

volatile int co2, gas, motion;

void reader() {
	co2 = analogRead(CO2PIN);
	gas = analogRead(GASPIN);
	motion = motionSensor.read();
	// todo replace strings with bytes
	Serial.print(co2);
	Serial.print(";");
	Serial.print(gas);
	Serial.print(";");
	Serial.println(motion);
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


	// Sensor status tasl
	runner.addTask(readSensors);

	// Starting
	runner.enableAll();
}

void loop() {
	runner.execute();
}
