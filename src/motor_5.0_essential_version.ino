/*
 * Features set
 * MOTOR .................
 * 1 controll all the motors
 * 2 Dry run protection
 * 3 Recurring motor ON OFF
 * 4 Low voltage protection
 *
 * IRRIGATION ............
 * 1 turn on drip irrigation system at specified time.
 * 2 controll the irrigation based on Soil Moisture.
 */
// include statements ........................................................
#include <Arduino.h>
#include "lib.h"
#include <RTClib.h>
#include <esp_task_wdt.h>
#include "EmonLib.h"

// pin definitions ...........................................................
#define WDT_TIMEOUT 10 // watchdog timer seconds.
#define relayoff HIGH
#define relayon LOW
#define dryrun 32
#define sumplevel 33
#define tankhigh 25       // green white color cable
#define tanklow 26        // orange color cable and green common ground
#define motor_1_on 27     // submersible motor on trigger
#define motor_1_off 13    // submersible motor off trigger
#define motor_2 14        // small motor
#define voltage_sensor 35 // Input Only
// #define current_sensor 39 // Input Only
#define debug_led 2       // for debugging purpose
#define plantrelay_1 19
#define plantrelay_2 18
#define soilsensor 36 // Input Only  working only on 36 and 39 pin 
// #define DHT11_PIN 34  // Input Only
#define voltage_threshold 190
#define soil_threshold 50

// Features .....................
#define Dryrun_Enable 0          // 1 = DryRun Protection ON || 0 = DryRun Protection OFF
#define voltage_sensing_enable 0 // 1 = Enable low voltage detection || 0 = Disable low voltage detection
#define Recurring_on 1           // 1 = Recurring motor ON || 0 = Recurring motor OFF
#define irrigation_enable 0      // 1 = Plant irrigation On || 0 = Plant irrigation OFF
#define moisture_sensor_enable 0 // 1 = Enable soil moisture sensor || 0 = Disable soil moisture sensor


#if Dryrun_Enable
unsigned long start_time;
#endif
#if voltage_sensing_enable
EnergyMonitor emon1;
#endif

bool var_motor_1 = false;
bool var_motor_2 = false;
bool plant_irrigation = false;
bool isdelay = false;

unsigned long currenttime = 0; //used to turn off the motor after specified time
unsigned long lock_ontime = 0; // lock the motor until Resting period

RTC_DS3231 rtc;

void setup()
{

  Serial.begin(115200);
  delay(1000);
  Serial.println("\nBooting........\n");

  pinMode(sumplevel, INPUT_PULLUP);
  #if Dryrun_Enable
  pinMode(dryrun, INPUT_PULLUP);
  #endif
  #if voltage_sensing_enable
  emon1.voltage(voltage_sensor, 130, 1.7);
  #endif
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
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); //Used to update RTC module time.

  esp_task_wdt_init(WDT_TIMEOUT, true); // enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);
}

void loop()
{
  // Reset the the timer or the controller will get reset
  esp_task_wdt_reset();
  DateTime now = rtc.now();

  Serial.print(now.hour());Serial.print(":");Serial.println(now.minute());

  #if voltage_sensing_enable
    emon1.calcVI(20,2000);                // Calculate all. No.of half wavelengths (crossings), time-out  
    // float supplyVoltage = emon1.Vrms;     //extract Vrms into Variable
    // Serial.println(emon1.Vrms);
  #endif

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
      #if Dryrun_Enable
        start_time = millis();
      #endif
      var_motor_2 = true;
      var_motor_1 = false;
    }
  }

  // motor_2 OFF when sumplevel is low & motor_1 ON
  if ((digitalRead(sumplevel) == LOW) && (digitalRead(tanklow) == LOW) && (var_motor_1 == false) && (!voltage_low()) && (!isdelay))
  {
    delay(1000);
    if ((digitalRead(sumplevel) == LOW) && (digitalRead(tanklow) == LOW) && (var_motor_1 == false) && (!voltage_low()) && (!isdelay))
    {
      Serial.println("motor_2 OFF when sumplevel is low & motor_1 ON");
      digitalWrite(motor_2, relayoff);
      delay(2000);
      motor_1_onstate();
      #if Dryrun_Enable
        start_time = millis();
      #endif
      var_motor_2 = false;
      var_motor_1 = true;
    }
  }

  // Dry Run Protection
  #if Dryrun_Enable
    currenttime = now.unixtime();
    if ((millis() < start_time + 5000) && (var_motor_1 || var_motor_2))
    {
      if (digitalRead(dryrun) == LOW)
      {
        Serial.println("Dry Run Protection triggered");
        digitalWrite(motor_2, relayoff);
        motor_1_offstate();
        var_motor_1 = false;
        var_motor_2 = false;
        isdelay = true;
        lock_ontime = now.unixtime();
      }
    }
    if ((isdelay) && (currenttime - lock_ontime >= 300))
    {
      isdelay = false;
      Serial.println("isdelay made to false");
    }
  #endif

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

  // motor_2 off when sum level is low
  if ((digitalRead(sumplevel) == LOW) && (var_motor_2 == true))
  {
    Serial.println("motor_2 OFF when sumplevel is low");
    digitalWrite(motor_2, relayoff);
    var_motor_2 = false;
  }

  // motor_1 and motor_2 ON and OFF by RTC for 7 secs
  #if Recurring_on
    if ((var_motor_1 == false) && (var_motor_2 == false))
    {
      if ((now.hour() == 04 && now.minute() == 00 && now.second() > 50) || (now.hour() == 17 && now.minute() == 33 && now.second() > 50)) // 4am and 5pm time should be changed based on the power fluctuation
      {
        esp_task_wdt_reset();
        motor_1_onstate();
        Serial.println("motor_1 ON by RTC");
        delay(7000); // run the motor for 7 secs
        esp_task_wdt_reset();
        motor_1_offstate();
        Serial.println("motor_1 OFF by RTC");
        esp_task_wdt_reset();
        delay(8000); // wait for 8 sec
        if (digitalRead(sumplevel) == HIGH)
        {
          delay(1000);
          if (digitalRead(sumplevel) == HIGH)
          {
            esp_task_wdt_reset();
            digitalWrite(motor_2, relayon);
            Serial.println("motor_2 ON by RTC");
            delay(7000); // run the motor for 7 secs
            digitalWrite(motor_2, relayoff);
            Serial.println("motor_2 OFF by RTC");
          }
        }
      }
    }
  #endif

  #if irrigation_enable
    // Turn on the plant irrigation_2 upto 6 clock at morning from 6 clock in evening.
    // Serial.println(now.hour());
    if (((now.hour() >= 18) || (now.hour() <= 7)) && (!plant_irrigation) && (soil_moisture_low())) // time should be changed without interupting the recurring motor timer
    {
      Serial.println("Plant Irrigation Turned On");
      digitalWrite(plantrelay_1, relayon); // turn on irrigation.
      plant_irrigation = true;
    }
    else if ((plant_irrigation) && (!((now.hour() >= 18) || (now.hour() <= 7))))
    {
      Serial.println("Plant Irrigation Turned Off");
      digitalWrite(plantrelay_1, relayoff); // turn off irrigation.
      plant_irrigation = false;
    }
    else if (!soil_moisture_low()) // used to turn off the irrigation, after it is turned on. when it is Raining.
    {
      Serial.println("Plant Irrigation Turned Off Due to Wet Soil");
      digitalWrite(plantrelay_1, relayoff); // turn off irrigation.
      plant_irrigation = false;
    }
  #endif
  delay(1000);
}

bool soil_moisture_low() // Function to check soil moisture
{
  #if moisture_sensor_enable
  if (analogRead(soilsensor) > soil_threshold) // change the soil reading based on the environment
    return true;                                // return true if soil moisture is low
  else
    return false;
  #else
    return true;
  #endif
}

bool voltage_low() // Function to check voltage value
{
  #if voltage_sensing_enable
    if (emon1.Vrms < voltage_threshold) // change the voltage based on the environment
      return true;                      // return true when voltage is low
    else
      return false;
  #else
    return false;
  #endif
}

// Todo : complete voltage mesurement
// Todo : complete soil moisture mesurement
// Todo : implement recurring motor ON/OFF

// Testing.
// * Motor on and off with sump and tank level --- OK
// * Low Voltage Protection --- yet to check
// * Recurring Motor on and off --- OK
// * Plant irrigation --- yet to check
// * Plant irrigation with moisture sensor --- yet to check
// * Dryrun protection --- yet to check
