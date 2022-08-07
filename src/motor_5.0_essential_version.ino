/*
 * Features set
 * MOTOR .................
 * 1 controll all the motors
 * 2 Dry run protection
 * 3 Recurring motor ON OFF
 *
 * IRRIGATION ............
 * 1 turn on drip irrigation system at specified time.
 * 2 controll the irrigation based on humidity and temperature.
 */
// include statements ........................................................
#include <Arduino.h>
#include "lib.h"
#include <RTClib.h>
#include <esp_task_wdt.h>

// pin definitions ...........................................................
#define WDT_TIMEOUT 5 // watchdog timer seconds.
#define relayoff HIGH
#define relayon LOW
#define dryrun 32
#define sumplevel 33
#define tankhigh 25       // green white color cable
#define tanklow 26        // orange color cable and green common ground
#define motor_1_on 27     // submersible motor on trigger
#define motor_1_off 13    // submersible motor off trigger
#define motor_2 14        // small motor
#define voltage_sensor 39 // Input Only
#define current_sensor 35 // Input Only
#define debug_led 2       // for debugging purpose
#define plantrelay_1 19
#define plantrelay_2 18
#define soilsensor 34 // Input Only
#define DHT11_PIN 36  // Input Only

// rtc to pin 21 and 22 SDA and SCL

uint8_t soil_moisture; // Stores Soil moisture data
uint8_t voltage;       // Stores Voltage data
unsigned long start_time;

bool var_motor_1 = false;
bool var_motor_2 = false;
bool plant_irrigation_1 = false;

RTC_DS3231 rtc;

void setup()
{

  Serial.begin(115200);
  delay(1000);
  Serial.println("Booting........");

  pinMode(sumplevel, INPUT_PULLUP);
  pinMode(dryrun, INPUT_PULLUP);
  pinMode(tankhigh, INPUT_PULLUP);
  pinMode(tanklow, INPUT_PULLUP);
  pinMode(motor_1_off, OUTPUT);
  pinMode(motor_1_on, OUTPUT);
  pinMode(motor_2, OUTPUT);
  pinMode(plantrelay_1, OUTPUT);
  pinMode(plantrelay_2, OUTPUT);

  digitalWrite(motor_2, relayoff);
  digitalWrite(motor_1_on, relayoff);
  digitalWrite(motor_1_off, relayoff);
  digitalWrite(plantrelay_1, relayoff);
  digitalWrite(plantrelay_2, relayoff);

  if (!rtc.begin())
  {
    Serial.println("RTC is NOT running!");
  }
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));

  esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);
}

void loop()
{
  // Reset the the timer or the controller will get reset
  esp_task_wdt_reset();
  DateTime now = rtc.now();

  // motor_2 ON when sump level is high
  if ((digitalRead(tanklow) == LOW) && (digitalRead(sumplevel) == HIGH) && (var_motor_2 == false))
  {
    delay(1000);
    if ((digitalRead(tanklow) == LOW) && (digitalRead(sumplevel) == HIGH) && (var_motor_2 == false))
    {
      Serial.println("motor_2 ON when sump level is high");
      motor_1_offstate();
      delay(1000);
      digitalWrite(motor_2, relayon);
      start_time = millis();
      var_motor_2 = true;
      var_motor_1 = false;
    }
  }

  // motor_2 & motor_1 OFF when tank fills
  if ((digitalRead(tankhigh) == HIGH) && ((var_motor_2 == true) || (var_motor_1 == true)))
  {
    delay(1000);
    if ((digitalRead(tankhigh) == HIGH) && ((var_motor_2 == true) || (var_motor_1 == true)))
    {
      Serial.println("motor_2 & motor_1 OFF when tank fills");
      digitalWrite(motor_2, relayoff);
      motor_1_offstate();
      var_motor_1 = false;
      var_motor_2 = false;
    }
  }

  // motor_2 OFF when sumplevel is low & motor_1 ON
  if ((digitalRead(sumplevel) == LOW) && (digitalRead(tanklow) == LOW) && (var_motor_1 == false))
  {
    delay(1000);
    if ((digitalRead(sumplevel) == LOW) && (digitalRead(tanklow) == LOW) && (var_motor_1 == false))
    {
      Serial.println("motor_2 OFF when sumplevel is low & motor_1 ON");
      digitalWrite(motor_2, relayoff);
      delay(2000);
      motor_1_onstate();
      start_time = millis();
      var_motor_2 = false;
      var_motor_1 = true;
    }
  }

  // Dry Run Protection
  if ((millis() < start_time + 5000) && (var_motor_1 || var_motor_2))
  {
    if (digitalRead(dryrun) == LOW)
    {
      Serial.println("Dry Run Protection triggered");
      digitalWrite(motor_2, relayoff);
      motor_1_offstate();
      var_motor_1 = false;
      var_motor_2 = false;
    }
  }

  // motor_2 off when sum level is low
  if ((digitalRead(sumplevel) == LOW) && (var_motor_2 == true))
  {
    Serial.println("motor_2 OFF when sumplevel is low");
    digitalWrite(motor_2, relayoff);
    var_motor_2 = false;
  }

  // Turn on the plant irrigation_2 upto 6 clock at morning from 6 clock in evening.
  // Serial.println(now.hour());
  if (((now.hour() >= 18) || (now.hour() <= 7)) && (!plant_irrigation_1) && (!check_soil_moisture())) // time should be changed without interupting the recurring motor timer
  {
    Serial.println("Plant Irrigation Turned On");
    digitalWrite(plantrelay_1, relayon); // turn on irrigation.
    plant_irrigation_1 = true;
  }
  else if ((plant_irrigation_1) && (!((now.hour() >= 18) || (now.hour() <= 7))))
  {
    Serial.println("Plant Irrigation Turned Off");
    digitalWrite(plantrelay_1, relayoff); // turn off irrigation.
    plant_irrigation_1 = false;
  }
  else if (soil_moisture <= 50)
  {
    Serial.println("Plant Irrigation Turned Off Due to Wet Soil");
    digitalWrite(plantrelay_1, relayoff); // turn off irrigation.
    plant_irrigation_1 = false;
  }
}

bool check_soil_moisture() // Function to check soil moisture
{
  if (analogRead(soilsensor) <= 50) // change the soil reading based on the environment
    return true;
  else
    return false;
}

bool check_voltage() // Function to check voltage value
{
  if (analogRead(voltage_sensor) >= 190) // change the voltage based on the environment
    return true;
  else
    return false;
}

// TODO : convert the constant variabes like voltage sensor 190 to define statement.