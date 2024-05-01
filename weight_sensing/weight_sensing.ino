#include <DynamixelWorkbench.h>
#include "pitches.h"
#include <math.h>
#include <Servo.h>

#define S0 2
#define S1 3
#define S2 4
#define MUX_EN_1 5
#define MUX_EN_2 7

#define BDPIN_BUZZER 31
#define BDPIN_LED_USER_1 22
#define BDPIN_LED_USER_2 23
#define BDPIN_LED_USER_3 24
#define BDPIN_LED_USER_4 25

#define REF_GRAD 450.0f
#define SAMPLING_TIME 0.01f
#define CUTOFF_FREQ 1.5f

#define BAUDRATE 115200
#define BAUDRATE_TO_DXL 115200
#define ACV_X_ID 20
#define ACV_Y1_ID 21
#define ACV_Y2_ID 22

#define X1 -49.2f
#define X2 51.6f
#define Y1 -39.2f
#define Y2 71.3f

#define _CEN 0
#define _WEST 1
#define _EAST 2
#define _SOUTH 3
#define _NORTH 4
#define _SW 5
#define _SE 6
#define _NW 7
#define _NE 8

#define NOT_ON_THE_PLATFORM	0
#define ON_THE_PLATFORM	1
#define MOTOR_STOP	2

// EDF Configuration
#define PWM_MIN 1100
#define PWM_MAX 1940

// Must be pwm function pin
#define EDF_RUDD1 6 // receiver No. 4, left side con
#define EDF_RUDD2 9 // receiver No. 5
#define EDF_THRT1 10 // receiver No. 3
#define EDF_THRT2 11 // receiver No. 2

DynamixelWorkbench dxl_wb;

Servo edf_rudder1;
Servo edf_rudder2;
Servo edf_throttle1;
//Servo edf_throttle2;


int32_t plate1[6];
int32_t plate2[6];

int32_t plate1_filtered[6];
int32_t plate2_filtered[6];
int32_t prev_plate1[6];
int32_t prev_plate2[6];
float plate1_force[6];
float plate2_force[6];
float tau = 1.0f / (2.0f * 3.141592f * CUTOFF_FREQ);

float xpos1[6] = {-306.25f, -361.25f, -251.25f, -361.25f, -251.25f, -306.25f};
float xpos2[6] = {306.25f, 251.25f, 361.25f, 251.25f, 361.25f, 306.25f};
float ypos[6] = {120.0f, 50.0f, 50.0f, -50.0f, -50.0f, -120.0f};

float force_offset1[6];
float force_offset2[6];

float gain_x_sen1[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
float gain_x_sen2[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
float gain_y_sen1[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 3.0f};
float gain_y_sen2[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 3.0f};

float x_cop, y_cop;
float x_offset, y_offset;
int dirVector;

int led_pin_user[4] = {BDPIN_LED_USER_1, BDPIN_LED_USER_2, BDPIN_LED_USER_3, BDPIN_LED_USER_4};
unsigned int timerCnt = 0;
unsigned int checkCnt = 0;
unsigned int stopCheckCnt=0;
bool isCheckStart=false;
bool isOffsetRemoved=false;
bool stopCheck=false;
float offset_x, offset_y;

int statusFlag = 0;

HardwareTimer Timer(TIMER_CH1);

void beep()
{
	tone(BDPIN_BUZZER, NOTE_A5, 50);
	delay(100);
	noTone(BDPIN_BUZZER);
}

void beepTime(int ms)
{
	tone(BDPIN_BUZZER, NOTE_A5);
	delay(ms);
	noTone(BDPIN_BUZZER);
}

void swichingMUX(int sensorNum)
{
	switch (sensorNum)
	{
	case 0:
		digitalWrite(S2, 0);
		digitalWrite(S1, 0);
		digitalWrite(S0, 0);
		break;

	case 1:
		digitalWrite(S2, 0);
		digitalWrite(S1, 0);
		digitalWrite(S0, 1);
		break;

	case 2:
		digitalWrite(S2, 0);
		digitalWrite(S1, 1);
		digitalWrite(S0, 0);
		break;

	case 3:
		digitalWrite(S2, 0);
		digitalWrite(S1, 1);
		digitalWrite(S0, 1);
		break;

	case 4:
		digitalWrite(S2, 1);
		digitalWrite(S1, 0);
		digitalWrite(S0, 0);
		break;

	case 5:
		digitalWrite(S2, 1);
		digitalWrite(S1, 0);
		digitalWrite(S0, 1);
		break;
	}
}

int32_t getSingleFSR(uint8_t moduleNum, uint8_t fsrNum)
{
	int32_t res;

	swichingMUX(fsrNum);
	delayMicroseconds(400); // 0.4ms

	if (moduleNum == 0)
	{
		digitalWrite(MUX_EN_1, 0); // turn on the mux1
		digitalWrite(MUX_EN_2, 1); // turn off the mux2
		delayMicroseconds(300);	   // 0.3ms
		res = analogRead(A0);
	}
	else
	{
		digitalWrite(MUX_EN_1, 1); // turn off the mux1
		digitalWrite(MUX_EN_2, 0); // turn on the mux2
		delayMicroseconds(300);	   // 0.3ms
		res = analogRead(A1);
	}

	return res;
}

void getAllFSR()
{
	for (int i = 0; i < 6; i++)
	{
		plate2[i] = getSingleFSR(0, 5 - i);
	}

	for (int i = 0; i < 6; i++)
	{
		plate1[i] = getSingleFSR(1, i);
	}
}

void printFSRValues()
{
	for (int i = 0; i < 6; i++)
	{
		Serial1.print(plate1[i]);
		Serial1.print(",");
	}
	for (int i = 0; i < 5; i++)
	{
		Serial1.print(plate2[i]);
		Serial1.print(",");
	}
	Serial1.println(plate2[5]);
}

void printFSRValues_filtered()
{
	for (int i = 0; i < 6; i++)
	{
		Serial.print(plate1_filtered[i]);
		Serial1.print(plate1_filtered[i]);
		Serial.print(",");
		Serial1.print(",");
	}

	for (int i = 0; i < 5; i++)
	{
		Serial.print(plate2_filtered[i]);
		Serial1.print(plate2_filtered[i]);
		Serial.print(",");
		Serial1.print(",");
	}
	Serial.println(plate2_filtered[5]);
	Serial1.println(plate2_filtered[5]);
}

void filteringFSR()
{
	for (int i = 0; i < 6; i++)
	{
		plate1_filtered[i] = (tau * prev_plate1[i] + SAMPLING_TIME * plate1[i]) / (tau + SAMPLING_TIME);
		prev_plate1[i] = plate1_filtered[i];

		plate2_filtered[i] = (tau * prev_plate2[i] + SAMPLING_TIME * plate2[i]) / (tau + SAMPLING_TIME);
		prev_plate2[i] = plate2_filtered[i];
	}
}

void print_force()
{
	for(int i=0; i<6; i++)
	{
		Serial1.print(plate1_force[i]);
		Serial1.print(",");
	}

	for(int i=0; i<5; i++)
	{
		Serial1.print(plate2_force[i]);
		Serial1.print(",");
	}
	Serial1.println(plate2_force[5]);
}

void calculateCOP_kilogram()
{
	float fsum = 0.0f;

	for(int i=0; i<6; i++)
	{
		plate1_force[i] = massConverter(plate1_filtered[i]);
		plate2_force[i] = massConverter(plate2_filtered[i]);
	}

	for (int i = 0; i < 6; i++)
	{
		fsum += (plate1_force[i] + plate2_force[i]);
	}

	x_cop = 0;
	y_cop = 0;

	for (int i = 0; i < 6; i++)
	{
		x_cop += (xpos1[i]*(plate1_force[i]-force_offset1[i])*gain_x_sen1[i] + xpos2[i]*(plate2_force[i]-force_offset2[i])*gain_x_sen2[i]);
		y_cop += (ypos[i]*(plate1_force[i]-force_offset1[i])*gain_y_sen1[i] + ypos[i]*(plate2_force[i]-force_offset2[i])*gain_y_sen2[i]);
	}

	//Serial1.println(fsum);

	if (fsum < 5.0f)
	{
		x_cop = 0;
		y_cop = 0;
	}

	else
	{
		x_cop /= fsum;
		y_cop /= fsum;
	}
}

void printCOP()
{
	Serial1.print(x_cop);
	Serial1.print(",");
	Serial1.print(y_cop);
}

void printCOPln()
{
	Serial1.print(x_cop);
	Serial1.print(",");
	Serial1.println(y_cop);
}

void toggleLED(uint8_t ledNum)
{
	static uint8_t flag[4] = {0, 0, 0, 0};
	digitalWrite(led_pin_user[ledNum], (flag[ledNum] & 0x01));
	flag[ledNum] ^= 0xff;
}

void writeLED(uint8_t ledNum, uint8_t status)
{
	digitalWrite(led_pin_user[ledNum], status);
}

void onLED(uint8_t ledNum)
{
	digitalWrite(led_pin_user[ledNum], LOW);
}

void offLED(uint8_t ledNum)
{
	digitalWrite(led_pin_user[ledNum], HIGH);
}

void mTimerFunc() // 1ms
{
	timerCnt++;
	if(!(timerCnt % 10))
	{
		filteringFSR();
	}

	if(statusFlag==NOT_ON_THE_PLATFORM && isCheckStart==true)
	{
		checkCnt++;
		if(!(timerCnt % 50))
		{
			toggleLED(0);
		}
	}
	else
	{
		checkCnt=0;
		offLED(0);
	}

	if(stopCheck==true)
	{
		stopCheckCnt++;
	}
	else
	{
		stopCheckCnt=0;
	}
}

int checkPlatePressure(int statusFlag)
{
	float fsum1=0.0f, fsum2=0.0f;

	for(int i=0; i<6; i++)
	{
		fsum1 += (float)plate1_filtered[i];
		fsum2 += (float)plate2_filtered[i];
	}

	// Serial1.print(fsum1);
	// Serial1.print(",");
	// Serial1.print(fsum2);
	// Serial1.print("\t");

	if(statusFlag == NOT_ON_THE_PLATFORM)
	{
		if(fsum1 > 1500 && fsum2 > 1500)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		if(fsum1 < 200 && fsum2 < 200)
		{
			return 0;
		}
		else
		{
			return 1;
		}
	}
}

void offsetRemove()
{
	for(int i=0; i<6; i++)
	{
		force_offset1[i]=0.0f;
		force_offset2[i]=0.0f;
	}

	for(int j=0; j<40; j++)
	{
		for(int i=0; i<6; i++)
		{
			force_offset1[i] += massConverter(plate1_filtered[i]);
			force_offset2[i] += massConverter(plate2_filtered[i]);
		}
		toggleLED(0);
		delay(50);
	}
	
	offLED(0);

	for(int i=0; i<6; i++)
	{
		force_offset1[i] /= 40.0f;
		force_offset2[i] /= 40.0f;
	}
}

int myClassifier(float x, float y)
{
	if(sqrt(x*x+y*y) < 40)
	{
		return _CEN;
	}

	else
	{
		if((y>x) && (y>-x))
		{
			return _NORTH;
		}

		else if((y<x) && (y<-x))
		{
			return _SOUTH;
		}

		else if((y>x) && (y<-x))
		{
			return _WEST;
		}
		else
		{
			if(x>150) return _EAST;
			else return _CEN;
		}
	}
}

float massConverter(int32_t adcVal)
{
	float res, volt;
	float out;

	volt = ((float)adcVal/1024.0f)*3.3f;
	

	if(volt < 0.4f)
	{
		return 0.0f;
	}

	else
	{
		res = 3300.0f/volt - 1000.0f;
		out = 67090.0f * pow(res,-1.601f) + 0.07617f;

		// Serial1.print(res);
		// Serial1.print(",");
		// Serial1.println(out);
		return out;
	}
}

void setup()
{
	pinMode(MUX_EN_1, OUTPUT);
	pinMode(MUX_EN_2, OUTPUT);
	pinMode(S0, OUTPUT);
	pinMode(S1, OUTPUT);
	pinMode(S2, OUTPUT);

	pinMode(led_pin_user[0], OUTPUT);
	pinMode(led_pin_user[1], OUTPUT);
	pinMode(led_pin_user[2], OUTPUT);
	pinMode(led_pin_user[3], OUTPUT);

	digitalWrite(MUX_EN_1, 1); // turn off the mux1
	digitalWrite(MUX_EN_2, 1); // turn off the mux2
	digitalWrite(S0, 0);
	digitalWrite(S1, 0);
	digitalWrite(S2, 0);

	Serial.begin(115200);
	Serial1.begin(115200);

	Timer.stop();
	Timer.setPeriod(1000); // 1msec
	Timer.attachInterrupt(mTimerFunc);
	Timer.start();

	beep();
    beep();
    beep();

	delay(6000); // wait for esc init.
	Serial.println("IDLE");
	Serial1.println("IDLE");

	statusFlag = NOT_ON_THE_PLATFORM;
	isCheckStart = false;
	isOffsetRemoved = false;
}

void loop()
{
	getAllFSR();
	delay(50);

    printFSRValues_filtered();
}