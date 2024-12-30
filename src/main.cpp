#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"


// Links:
// https://michiel.vanderwulp.be/domotica/Modules/ESP32-C3-SuperMini/

// Pin Definitions
#define LED_ONBOARD 8

// I2C IMU definition
#define I2C_IMU_SDA 10
#define I2C_IMU_SCL 9

// SCH definitions
#define SCH_20MS_TASK_PERIOD   (20000)
#define SCH_100MS_TASK_PERIOD (100000)
#define SCH_500MS_TASK_PERIOD (500000)
#define SENS_SAMPLING_TIME (float)((float)SCH_20MS_TASK_PERIOD/1000000.0f);

// Global Variables

// Scheduler variables
uint32_t schTimeUs = 0;
uint32_t schLast20msTaskExecutionUs = 0;
uint32_t schLast100msTaskExecutionUs = 0;
uint32_t schLast500msTaskExecutionUs = 0;

// ESP-NOW variables
#define dataLength 8
uint8_t incomingData[dataLength] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t responseData[dataLength] = {0, 0, 0, 0, 0, 0, 0, 0};
bool dataReceived = false;
// Own MAC Address: 9c:9e:6e:f6:b9:40


// LED Flag
bool LEDFlag = true;
uint8_t LEDCounter = 0;


// Function Prototypes
void schRun();
void task_20ms();
void task_100ms();
void task_500ms();
void readMacAddress();


TwoWire I2CTOF = TwoWire(0);
Adafruit_VL53L0X tof = Adafruit_VL53L0X();


// TOF Sensor
bool TOF_available = false;

// Callback function that will be executed when data is received
void onDataRecv(const uint8_t *mac, const uint8_t *data, int len) {
    dataReceived = true;
    // Clear the receive buffer
    for (int i = 0; i < dataLength; i++) {
        incomingData[i] = 0;
    }
    
    // Copy the received data to the buffer up to dataLength
    Serial.print("RxD: ");
    for (int i = 0; (i < len) && (i < dataLength); i++)
    {
        incomingData[i] = data[i];
        Serial.print(data[i]);
        Serial.print(" ");
    }
    Serial.println();
}


/* **********************************************************/
/* ******************** Setup Finction ******************** */
/* **********************************************************/
void setup()
{
  // Initialize the LED pin as an output
  pinMode(LED_ONBOARD, OUTPUT);
  digitalWrite(LED_ONBOARD, 0);
  
  // Initialize the Serial port
  Serial.begin(921600);
  // Wait for serial to initialize
  while(0 >= Serial.read())
  {
    delay(10);
  }

  // Initialize the I2C bus for Time of Flight sensor
  I2CTOF.begin(I2C_IMU_SDA, I2C_IMU_SCL, 400000);

  // Initialize the Time of Flight sensor
  TOF_available = tof.begin(0x29, false, &I2CTOF);
  if (!TOF_available)
  {
    Serial.println(F("Failed to boot VL53L0X"));
  }
  else
  {
    Serial.println(F("VL53L0X Booted"));
  }

  // Initialize WIFI in station mode
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // Initialize the ESP-NOW as a Slave
  if(esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  else
  {
    Serial.println("ESP-NOW Initialized");
  }
  
  // Register the callback function to be called when data is received
  esp_now_register_recv_cb(onDataRecv);

}


/* *************************************************** */
/* ******************** Main Loop ******************** */
/* *************************************************** */
void loop()
{
  // run scheduler
  schRun();
}


/* *************************************************** */
/* ******************** Scheduler ******************** */
/* *************************************************** */
void schRun()
{
  schTimeUs = micros();
  if ((schTimeUs - schLast20msTaskExecutionUs) >= SCH_20MS_TASK_PERIOD)
  {
    task_20ms();
    schLast20msTaskExecutionUs += SCH_20MS_TASK_PERIOD;
  }
  else if((schTimeUs - schLast100msTaskExecutionUs) >= SCH_100MS_TASK_PERIOD) {
    task_100ms();
    schLast100msTaskExecutionUs += SCH_100MS_TASK_PERIOD;
  }
  else if ((schTimeUs - schLast500msTaskExecutionUs) >= SCH_500MS_TASK_PERIOD)
  {
    task_500ms();
    schLast500msTaskExecutionUs += SCH_500MS_TASK_PERIOD;
  }
}


/* **************************************************** */
/* ********************* 20ms Task ******************** */
/* **************************************************** */
void task_20ms()
{
  if(dataReceived)
  {
    Serial.println("Data Received");
    Serial.print("Data: ");
    for(int i = 0; i < dataLength; i++)
    {
      Serial.print(incomingData[i]);
      Serial.print(" ");
    }
    Serial.println();
    dataReceived = false;
    LEDCounter = incomingData[0] * 20;
  }

  if(LEDCounter)
  {
    digitalWrite(LED_ONBOARD, 0);
    LEDCounter--;
  }
  else
  {
    digitalWrite(LED_ONBOARD, 1);
  }



/*
  if(incomingData[0] == 0x01)
  {
    Serial.println("Sending Response");
    esp_now_send(NULL, responseData, 8);
  }
*/
}


/* **************************************************** */
/* ******************** 100ms Task ******************** */
/* **************************************************** */
void task_100ms()
{
  VL53L0X_RangingMeasurementData_t measure;

  if(TOF_available)
  {
    Serial.print("Reading a measurement... ");
    tof.rangingTest(&measure, false); // pass in 'true' to get debug data printout!

    if (measure.RangeStatus != 4)   // phase failures have incorrect data
    {
      Serial.print("Distance (mm): "); Serial.println(measure.RangeMilliMeter);
    }
    else
    {
      Serial.println(" out of range ");
    }
  }

}


/* **************************************************** */
/* ******************** 500ms Task ******************** */
/* **************************************************** */
void task_500ms()
{
  LEDFlag = !LEDFlag;
  digitalWrite(LED_ONBOARD, ((LEDFlag == true) ? 0:1) );
  //readMacAddress();
}


void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}