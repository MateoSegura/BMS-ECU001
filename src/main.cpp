/* 
* Billet Motorsport
* ECU001
* REV: 1.0
* Compatible with ECU001-REV1.0 Hardware
* Last Modified: 12/20/20
*/

//TODO: GET BASIC APP WORKING
//TODO: 0-5V OR 0.5 TO 4.5V feature
//TODO: GET TEMPERATURE READING
//TODO: BLUETOOTH DEBUGGING: VOLTAGE FREQUENCY PULSE WIDTH 
//TODO: CRASH REPORTS AND TIME STAMPING?

/****************************************************************LIBRARIES****************************************************************/
#include <Arduino.h>
#include <cmath>
#include <iostream>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_MCP4725.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/**************************************************************DEFINTIONS*************************************************************/

//BLE Service Characteristics
#define BLENAME                "Billet Motorsport ECU001" 
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

uint64_t current_sample_time;
uint64_t last_sample_time;
uint8_t  sample_frequency = 10;    //Hz

//Serial debugging
bool serial_debugging = true;
bool bluetooth_debugging = true;

//IO
#define ETHANOLIN 25
#define NEOPIXEL  27

//BLE
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;

//DAC
Adafruit_MCP4725 dac;

//LED
Adafruit_NeoPixel pixels(1, NEOPIXEL, NEO_GRB + NEO_KHZ800);
uint8_t brightness = 100;

/**************************************************************Variables*************************************************************/

volatile uint64_t StartValue;                     // First interrupt value
volatile uint64_t PeriodCount;                    // period in counts of 0.000001 of a second
float             frequency;                           // frequency     
char              str[21];                        // for printing uint64_t values
float temperature; 

hw_timer_t * timer = NULL;                        // pointer to a variable of type hw_timer_t 
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;  // synchs between maon cose and interrupt?

/**************************************************************Function Defintions*************************************************************/

//BLE
void handleBluetoothMessage(std::string data);
void connectionManager();

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if(bluetooth_debugging){
        handleBluetoothMessage(rxValue);
      }
    }
};

//Ethanol In
void getFrequency(); 
void ethanolSensorHandle();
void IRAM_ATTR handleInterrupt(); //Ethanol Sensor ISR


/**************************************************************Setup*************************************************************/

void setup() 
{ 
  //DAC
  dac.begin(0x60);
  dac.setVoltage(0,false);   //Set default voltage to 0
  
  //Serial
  if(serial_debugging){
    Serial.begin(115200);
  }

  //Ethanol
  pinMode(26, INPUT_PULLUP);  //EN
  pinMode(ETHANOLIN, INPUT_PULLUP);                                            
  attachInterrupt(digitalPinToInterrupt(ETHANOLIN), handleInterrupt, FALLING); 
  timer = timerBegin(0, 80, true);                                                
  timerStart(timer);                                                              

  //BLE
  BLEDevice::init(BLENAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
										  CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
									);
                      
  pTxCharacteristic->addDescriptor(new BLE2902());
  pTxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();
  pServer->getAdvertising()->start();
  
  if(serial_debugging){
    Serial.println("[BLE] BLE Service is Advertsing and waiting for a connection to notify.");
  }

  //LED
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 0, brightness));
  pixels.show();
}


/*********************************************************loop*************************************************************/

void loop() 
{
  //Manage BLE Connection
  connectionManager();

  //Interval Calculation
  current_sample_time = millis();

  if((current_sample_time - last_sample_time) > (sample_frequency*10)){

    last_sample_time = current_sample_time;

    //Ethanol Reading
    portENTER_CRITICAL(&mux);
    frequency =1000000.00/PeriodCount;                       // PeriodCount in 0.000001 of a second
    portEXIT_CRITICAL(&mux);

    if(frequency >= 0 && frequency <= 150){
      frequency = frequency;
    }else if(frequency < 0){
      frequency = 0;
    }else if(frequency > 150){
      frequency = 150;
    }

    //Convert frequency to integer for easier processing
    int frequency_int = round(frequency);

    //Output Voltage
    float voltage_dac = map(round(frequency),0,100,0,4095);
    

    if(voltage_dac >= 0 && voltage_dac <= 4095){
      voltage_dac = voltage_dac;
    }else if(voltage_dac < 0){
      voltage_dac = 0;
    }else if(voltage_dac > 4095){
      voltage_dac = 4095;
    }

    //Approximate Voltage Output reading
    float voltage = map(voltage_dac,0,4095,0,5000);

    if(serial_debugging){
      Serial.print("[SERIAL] Data: "); Serial.print("Frequency: "); Serial.print(round(frequency)); Serial.print("Voltage: "); Serial.println(voltage/1000);
    }

    dac.setVoltage(voltage_dac,false);

    //Compose BLE Messgae
    if(deviceConnected){
      String ble_data;
      ble_data += frequency_int;
      ble_data += ",";
      ble_data += temperature;
      ble_data += ",";
      ble_data += voltage;

      pTxCharacteristic->setValue((char*)ble_data.c_str());
      pTxCharacteristic->notify();
    }
  }
}

/*********************************************************FUNCTION IMPLEMENTATION*************************************************************/

void handleBluetoothMessage(std::string data){
  if (data.length() > 0) {
        Serial.println("*********");
        Serial.print("Received Value: ");
        for (int i = 0; i < data.length(); i++)
          Serial.print(data[i]);

        Serial.println();
        Serial.println("*********");
  }
}

void connectionManager(){

  // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        if(serial_debugging){
          Serial.println("[BLE] BLE Service has disconnected.");
        }
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(0, 0 , brightness));   //Green if connected
        pixels.show();
        delay(200);
        pServer->startAdvertising(); // restart advertising
        if(serial_debugging){
          Serial.println("[BLE] BLE Service is Advertsing and waiting for a connection to notify.");
        }
        oldDeviceConnected = deviceConnected;
    }

    // connecting
    if (deviceConnected && !oldDeviceConnected) {
        oldDeviceConnected = deviceConnected;
        pixels.clear();
        pixels.setPixelColor(0, pixels.Color(0, brightness , 0));   //Green if connected
        pixels.show();
        if(serial_debugging){
          Serial.println("[BLE] Device Connected to ");
        }
    }
}

void IRAM_ATTR handleInterrupt() 
{
  portENTER_CRITICAL_ISR(&mux);
      uint64_t TempVal= timerRead(timer);         // value of timer at interrupt
      PeriodCount= TempVal - StartValue;          // period count between rising edges in 0.000001 of a second
      StartValue = TempVal;                       // puts latest reading as start for next calculation
  portEXIT_CRITICAL_ISR(&mux);
}
