/* 
* Billet Motorsport
* ECU001
* REV: 1.0
* Compatible with ECU001-REV1.0 Hardware
* Last Modified: 12/20/20
*/

//TODO: 0-5V OR 0.5 TO 4.5V feature
//TODO: GET TEMPERATURE READING
//TODO: OTA
//TODO: UUID
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

// /**************************************************************DEFINTIONS*************************************************************/

//BLE Service Characteristics
#define BLENAME                "Billet Motorsport ECU001" 
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

uint64_t current_sample_time;
uint64_t last_sample_time;
uint8_t  sample_frequency = 10;    

//Active
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
bool output_mode = 1;  //0: 0 TO 5V, 1:0.2 TO 4.5V

//LED
Adafruit_NeoPixel pixels(1, NEOPIXEL, NEO_GRB + NEO_KHZ800);
uint8_t brightness = 100;

/**************************************************************Variables*************************************************************/

bool state = false;                                //Falling
volatile uint64_t last_rising_time;                     
volatile uint64_t current_rising_time;        
volatile uint64_t fallinging_time;   
volatile uint64_t period_count; 
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
void IRAM_ATTR initialInterrupt();
void IRAM_ATTR handleInterrupt(); //Ethanol Sensor ISR


/**************************************************************Setup*************************************************************/

void setup() 
{ 
  Serial.begin(115200);
  //LED
  pixels.begin();
  pixels.clear();
  pixels.setPixelColor(0, pixels.Color(0, 0, brightness));
  pixels.show();

  //DAC
  dac.begin(0x60);
  dac.setVoltage(0,true);   //Set default voltage to 0
  
  //Serial
  if(serial_debugging){
    Serial.begin(115200);
  }                                                             

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

    //pService->start();
    //pServer->getAdvertising()->start();
    
    if(serial_debugging){
      Serial.println("[BLE] BLE Service is Advertsing and waiting for a connection to notify.");
    }

  //Ethanol
  pinMode(26, INPUT_PULLUP);  //EN
  pinMode(ETHANOLIN, INPUT_PULLUP);                                     
  attachInterrupt(digitalPinToInterrupt(ETHANOLIN), initialInterrupt, FALLING); 
  timer = timerBegin(0, 80, true);                                               
  timerStart(timer); 
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

    //Temporary Variables
    float _last_rising_time, _current_rising_time, _falling_time;

    //Ethanol Reading
    portENTER_CRITICAL(&mux);

    _current_rising_time = current_rising_time;
    _falling_time = fallinging_time;
    _last_rising_time = last_rising_time;
    frequency =1000000.00/period_count;        

    portEXIT_CRITICAL_ISR(&mux); 

    //Get Wave's pulse width in MS
    double pulse_width = (_falling_time - _current_rising_time)/1000;

    if(pulse_width > 0){

      //Convert frequency to integer for easier processing
      int frequency_int = round(frequency);

      //Output Voltage
      float voltage_dac;

      portENTER_CRITICAL_ISR(&mux);
      if(output_mode){

        //0.5 to 4.5V
        voltage_dac = map(round(frequency),50,150,0,4095);

        if(voltage_dac >= 0 && voltage_dac <= 4095){
          voltage_dac = voltage_dac;
        }else if(voltage_dac < 0){
          voltage_dac = 0; 
        }else if(voltage_dac > 4095){
          voltage_dac = 4095;
        }

      }else{
        //0 to 5V
        voltage_dac = map(round(frequency),50,150,410,3700);

        if(voltage_dac >= 150 && voltage_dac <= 3700){
          voltage_dac = voltage_dac;
        }else if(voltage_dac < 410){
          voltage_dac = 410; 
        }else if(voltage_dac > 3700){
          voltage_dac = 3700;
        }

      }
      portEXIT_CRITICAL_ISR(&mux);
    

      

      //Approximate Voltage Output reading
      float voltage = map(voltage_dac,0,4095,0,4950);
      int voltage_int = round(voltage);

      if(serial_debugging){
        Serial.print("[SERIAL] Data: "); Serial.print("Frequency: "); Serial.print(round(frequency)); Serial.print(" Pulse Width: "); Serial.print(pulse_width); Serial.print("Voltage: "); Serial.println(voltage/1000);
      }

      //Set DAC Output
      dac.setVoltage(voltage_dac,false);

      //Compose BLE Messgae
      if(deviceConnected){
        String ble_data;
        ble_data += frequency_int;
        ble_data += ",";
        ble_data += pulse_width;
        ble_data += ",";
        ble_data += voltage_int;

        pTxCharacteristic->setValue((char*)ble_data.c_str());
        pTxCharacteristic->notify();
      }
    }
       
    
  }
}

/*********************************************************FUNCTION IMPLEMENTATION*************************************************************/

void handleBluetoothMessage(std::string data){
  if (data.length() > 0) {
    switch (data[0])
    {
    case 'O':
      if(data[1] == '1'){
        //Change mode to 0 to 5V
        Serial.println("0 to 5V output");
        portENTER_CRITICAL_ISR(&mux);
        output_mode = 1;
        portEXIT_CRITICAL_ISR(&mux);
      }else if(data[1] == '0'){
        //Change mode to 0.5 to 4.5V
        Serial.println("0.5 to 4.5V output");
        portENTER_CRITICAL_ISR(&mux);
        output_mode = 0;
        portEXIT_CRITICAL_ISR(&mux);
      }
      break;
    default:
      break;
    }
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
  if(state){
    state = !state;
    last_rising_time = current_rising_time;
    current_rising_time = timerRead(timer);
    period_count = current_rising_time - last_rising_time;
  }else{
    state = !state;
    fallinging_time = timerRead(timer);
  }
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR initialInterrupt(){
  detachInterrupt(ETHANOLIN);
  attachInterrupt(digitalPinToInterrupt(ETHANOLIN), handleInterrupt, CHANGE);
}
