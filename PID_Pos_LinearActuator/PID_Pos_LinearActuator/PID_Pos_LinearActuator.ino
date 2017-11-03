#include <PID_v1.h>
#define pull	2                       // PWM outputs to L298N H-Bridge motor driver module
#define push	3
#define enable	9

double kp = 5, ki = 1, kd = 0.01;             // modify for optimal performance
double Setpoint, Input, Output;
const int SampleTime = 1;
const int minPWM = -255;
const int maxPWM = 255;
long temp;

PID myPID(&Input, &Output, &Setpoint, kp, ki, kd, DIRECT);

void setup()
{
	TCCR1B = TCCR1B & 0b11111000 | 1;	//	Set 31KHz PWM to prevent motor noise
	Input = analogRead(A0);
	Setpoint = 500;

	myPID.SetMode(AUTOMATIC);
	myPID.SetSampleTime(SampleTime);
	myPID.SetOutputLimits(minPWM, maxPWM);
	digitalWrite(enable, HIGH);

	pinMode(pull, OUTPUT);	//When HIGH the actuator push
	pinMode(push, OUTPUT);	//When HIGH the actuatot pull
	Serial.begin(9600);		//Serial initialization
}

void loop()
{
	//setpoint = 760;
	Input = analogRead(A0);	// data from encoder
	Serial.println(Input);	// monitor motor position
	myPID.Compute();	// calculate new output
	pwmOut(Output);	// drive L298N H-Bridge module
}

//Signal to the H-Bridge board
void pwmOut(int out)
{
	if (out > 0)
	{
		analogWrite(pull, out);		//Pulling linear actuator
		analogWrite(push, LOW);
	}
	else
	{
		analogWrite(pull, LOW);
		analogWrite(push, abs(out));	//Pushing linear actuator
	}
}