/*
Name:    Thermo.ino
Created: 8/27/2018 8:51:46 PM
Author:  Marian
*/

/*
Features:
*maintain water temperature at a set value
*turn off under a certain threshold
*LCD light on or off
*output power measurement (kW)
*energy used in the last 12h (kWh)
*/
//built using Visual Micro
#pragma GCC push_options
#pragma GCC optimize ("O3")
#include <TimerOne.h>			//using https://github.com/PaulStoffregen/TimerOne
#include <math.h>
#include <LiquidCrystal.h>
//PINS:
//pins used for the LCD interface
const uint8_t rs = 2, en = 3, d4 = 9, d5 = 8, d6 = 7, d7 = 6;

//hot water thermistor pin, Analog
const uint8_t hot_input = A2;
//cold water thermistor pin, Analog
const uint8_t cold_input = A1;

//value input, Analog
const uint8_t pot_pin = A0;

//relay control pin, Digital
const uint8_t relay_pin = 10;

const uint8_t backlight = 11;

const uint8_t led_running = 13;

//number of values samples before using the average
const uint8_t n_samples = 3;

//Globals:

//LCD object init
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

//threshold to be set by the pot
uint16_t threshold;

int temp_hot, temp_cold;  //temperature in deg. C

bool running=false;			//pump running or not
bool led_state = false;

//timing variables
unsigned long lastUpdate = 0;

//sampling vars
uint16_t samples_hot[10], samples_cold[10], samples_threshold[10];

//power output
float power = 0;

//bool for display purposes: + = 1, x = 2
uint8_t show_mode = 0;

//custom chars for the LCD
byte degree[8] = {
	B00111,
	B00101,
	B00111,
	B00000,
	B00000,
	B00000,
	B00000
};
byte running_symbol[2][8] = { {
	B00000,

	B00100,
	B00100,
	B11111,
	B00100,
	B00100,

	B00000
},{
	B00000,

	B10001,
	B01010,
	B00100,
	B01010,
	B10001,

	B00000
} };
//temperature map. temp[resistance in kOhms] = temperature;
const float temp[101] = {159.9, 152.2, 144.9, 138, 131.4, 125.2, 119.4, 113.8, 108.5, 103.5,
					98, 942, 89.9, 85.8, 81.9, 78.2, 74.7, 71.4, 68.2, 65.2,
					62.3, 59.6, 57, 54.5, 52.2, 50, 47.8, 45.8, 43.8, 42, 40.2,
					38.5, 36.9, 35.4, 33.9, 32.6, 31.2, 29.9, 28.7, 27.6, 26.5,
					25.4, 24.4, 23.4, 22.5, 21.6, 20.8, 20, 19.2, 18.4, 17.9,
					17.1, 16.4, 15.8, 15.2, 14.6, 14.1, 13.5, 13, 12.6, 12.1, 
					11.7, 11.2, 10.8, 10.4, 10, 9.7, 9.4, 9, 8.7, 8.4, 
					8.1, 7.8, 7.5, 7.3, 7, 6.8, 6.5, 6.3, 6.1, 5.9, 
					5.7, 5.5, 5.3, 5.1, 5, 4.8, 4.7, 4.5, 4.4, 4.2, 
					4.1, 3.9, 3.8, 3.7, 3.6, 3.5, 3.4, 3.3, 3.2, 3.1};
void configureIO() {
	Timer1.initialize(25);		//25us = 40KHz
	analogReference(EXTERNAL);	//external 2.5V reference for the thermistors and threshold pot
	pinMode(hot_input, INPUT);
	pinMode(cold_input, INPUT);
	pinMode(pot_pin, INPUT);
	pinMode(relay_pin, OUTPUT);
	pinMode(backlight, OUTPUT);
	pinMode(led_running, OUTPUT);
}

void startPump() {
	if (!running) {
		running = true;
		Timer1.pwm(relay_pin, 100);	//40KHz @ 10% duty cycle to the gate drive transformer
		digitalWrite(backlight, HIGH);
	}
}

void stopPump() {
	if (running) {
		running = false;
		Timer1.pwm(relay_pin, 0);	//0% duty cycle
		digitalWrite(backlight, LOW);
	}
}

void warnTemperature() {
	digitalWrite(backlight, HIGH);
	delay(300);
	digitalWrite(backlight, LOW);
	delay(300);
}

int getTempFromTable(float resistance) {
	/*
		Find the closest aproximation of the temperature from the temp map
		resistance given in Ohms
	*/
	
	resistance /= 1000;
	int middle, left = 0, right = 100;
	while (true) {
		middle = (left + right) / 2;

		if (left - right <= 4) {
			float min_delta = 99;
			int idx;
			for (int i = left; i <= right; i++) {
				float delta = abs(resistance - temp[i]);
				if (delta < min_delta) {
					min_delta = delta;
					idx = i;
				}
			}
			return idx;
		}
		else if (temp[middle] >= resistance) {
			//search in the right half
			left = middle;
		}
		else {
			//search in the left half
			right = middle;
		}
	}
}

void getTemperature() {
	/*
	convert avg_cold, avg_hot to deg. C using the Stein-Hart equation
	*/
	//read the voltage and compute the resistance

	double r_hot, r_cold;
	double Vout_hot, Vout_cold;
	double Vref_intern = 5;	//used internally, 5V=1023
	double Vref = 4.52;	//thermistor voltage
	double Rg_hot = 9560, Rg_cold=9560;	//resistance of the pull down resistors for the hot and cold thermistors
	//get average
	uint16_t avg_hot = 0;
	uint16_t avg_cold = 0;
	for (int i = 0; i < n_samples; i++) {
		avg_hot += samples_hot[i];
		avg_cold += samples_cold[i];
	}
	avg_hot /= n_samples;
	avg_cold /= n_samples;

	Vout_hot = avg_hot / 1023.0F*Vref;

	Vout_cold = avg_cold / 1023.0F*Vref;
	//for 10k Rg
	r_hot = (Rg_hot*Vref) / Vout_hot - Rg_hot;
	r_cold = (Rg_cold*Vref) / Vout_cold - Rg_cold;

	//get the temperature from resistance and convert to Celsius
	temp_hot = getTempFromTable(r_hot);
	temp_cold = getTempFromTable(r_cold);
}

void getPower() {
	/*
	Get the power output and update the energy used
	*/
	//the flow of the pump in kg/s
	float flow = 0.05;
	power = (temp_hot - temp_cold)*4.2*flow;
}

void updateDisplay() {
	/*
	Update the display with current temperature, power, threshold and the running symbol	
	T:47*C P:24.4kW
	Total: 1112kWh +
	*/
	lcd.clear();
	lcd.setCursor(0, 0);
	lcd.print("T:");
	lcd.print(temp_hot);
	lcd.write(byte(0));
	lcd.print("C  ");
	lcd.print("P:");
	lcd.print(power, 1);
	lcd.print("kW");
	lcd.setCursor(0, 1);
	lcd.print("Limita: ");
	lcd.print(threshold);
	lcd.write(byte(0));
	lcd.print("C ");
	lcd.print("  ");
	lcd.write(byte(show_mode));
	//toggle the running symbol when the pump is started
	show_mode == 1 && running ? show_mode = 2 : show_mode = 1;
}

void setup() {
	lcd.createChar(0, degree);
	lcd.createChar(1, running_symbol[0]);
	lcd.createChar(2, running_symbol[1]);
	lcd.begin(16, 2);
	lcd.print("Configuring...");
	configureIO();
}

// the loop function runs over and over again until power down or reset
void loop() {

	//threshold temperature from 25 to 85 degrees C
	threshold = map(analogRead(pot_pin), 0, 1023, 25, 85);

	if (millis() - lastUpdate > 700) {

		getTemperature();
		getPower();
		//update display
		updateDisplay();
		if (temp_hot > 90) {
			warnTemperature();
		}

		if (temp_hot <= threshold) {
			stopPump();
		}
		else {
			startPump();
		}
		//toggle the LED
		digitalWrite(led_running, led_state);
		led_state != led_state;

		lastUpdate = millis();
	}
	else {
		//get samples
		for(int sample=0; sample<n_samples; sample++) {
			samples_cold[sample] = analogRead(hot_input);
			samples_hot[sample] = analogRead(cold_input);
			samples_threshold[sample] = analogRead(pot_pin);
			delay(100);
		}
	}
}

#pragma GCC pop_option
