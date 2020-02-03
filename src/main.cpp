/*
MIT License
Copyright (c) 2020 mrbubble62
Source: https://github.com/mrbubble62/ESP-Humidifer 
Humidifier controller
WiFi Setup: Connect to the the HUMIDIFIER AP on first start, select your WiFi AP and enter your password.
Change the Over The Air (OTA) firmware update password if required.
*/

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <Hash.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <FS.h>
#include <EEPROM.h>
#include <PID_v1.h>
#include <ArduinoOTA.h>
#include <DNSServer.h>
#include <ESPAsyncWiFiManager.h>  

#define EEPROM_ADDR_CONFIG 1 // starting eeprom address for config params
#define MIN_SWITCH_T 15 //Relay On/Off minimum time in seconds

#ifndef HOST_NAME // check platfomio.ini
#define HOST_NAME "humidity"
#endif

const char* password = "mrbubble"; // WifiManager access password
const char* hostname = HOST_NAME;  // name device will try to register in DNS
char otapassword[33] = "OTApassword"; //default OTA password (can be changed in web setup)

String OTApassword = otapassword;
AsyncWebServer server(80);
DNSServer dns;
bool shouldSaveConfig = false;

// software I2C setup
Adafruit_BME280 bme;
bool bmeStatus;

// Relay 1 used to control the humidifier - used in normally open (NO) mode
// which means when it is set to 'HIGH', the humidifier will turn on
const byte RelayPin = D1;
bool powerOn;

 // 500 ms loop
uint32_t delt_t = 0; 
uint32_t count = 0;

 // > 0 = delay before updating eeprom in loop iterations
int configChange=0;

// status
volatile double bmeTemp=25.0;
volatile double bmeRH=33.3;
bool relayOn;
unsigned long windowStartTime;

//PID
double Setpoint=40.0, Input, Output; // Variable names for PID
double Error;
// Define window size 
unsigned long WindowSize = 90000;  // Cycle on and off every 90 seconds

// PID coefficients
double Kp = 2.0;  // Proportional: Multiplier of raw error (tune first)
double Ki = 1.0;  // Integral: Multiplier of sum of error over time (tune second)
double Kd = 0.1;  // Differential: Multiplier of rate that error is changing (tune third)

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

//EEPROM
const uint16_t MAGIC =  0xee11;
struct tConfig {
	uint16_t Magic; //test if eeprom initialized
	double P;
	double I;
	double D;
  double Setpoint;
  unsigned long WindowSize;
  String OTApassword;
};
// EEPROM contents
tConfig config;

// Default EEPROM contents
const tConfig defConfig = {
	MAGIC,
  Kp,
	Ki, 
	Kd,  
	Setpoint, 
  WindowSize,
  otapassword
};

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void UpdateConfig() {
  config.P=Kp;
  config.I=Ki;
  config.D=Kd;
  config.Setpoint=Setpoint;
  config.WindowSize=WindowSize;
  config.OTApassword=otapassword;
  EEPROM.put(EEPROM_ADDR_CONFIG, config);
  EEPROM.commit();
  Serial.println("\nsaved.");
}

void ReadConfig() {
  EEPROM.begin(512);
 	EEPROM.get(EEPROM_ADDR_CONFIG, config);
  if (config.Magic != MAGIC) {
    Serial.println(F("No stored calibration..\n"));
    config = defConfig;
    UpdateConfig();
  }
 	Kp = config.P;    
	Ki = config.I; 
	Kd = config.D;
	Setpoint = config.Setpoint;
	WindowSize = config.WindowSize;
}

void printConfig(){
  Serial.printf("Kp: %f ",Kp);
  Serial.printf("Ki: %f ",Ki);
  Serial.printf("Kd: %f ",Kd);
  Serial.printf("SP: %f ",Setpoint);
  Serial.printf("T: %lu \n",WindowSize);
}

//serial input
char getCommand(){
	char c = '\0';
	if (Serial.available())	{
		c = Serial.read();
	}
	return c;
}

void ReadSensor(){
  if(bmeStatus){
    bmeTemp = bme.readTemperature(); 
    bmeRH = bme.readHumidity();
    bme.takeForcedMeasurement();
  }
}

void notFound(AsyncWebServerRequest *request) {
    request->send(404, "text/plain", "Not found");
}

void setup() {
  powerOn=true;
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(100); 

  Serial.println("Start OTA");
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(otapassword);

  ArduinoOTA.onStart([]() {
    Serial.println("Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("OTA ready");
  
  //read stored config (including otapassword)
  Serial.println("Read stored configuration");
  ReadConfig();
  printConfig();

  // WiFi Manager
  Serial.println("Connecting to WiFi");
  AsyncWiFiManager wifiManager(&server,&dns);
  AsyncWiFiManagerParameter set_otapassword("otapassword", "OTA Password", otapassword, 34);
  //set config save notify callback
  WiFi.hostname(hostname);
  wifiManager.setDebugOutput(false);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&set_otapassword);
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setMinimumSignalQuality(20);
  wifiManager.autoConnect("HUMIDIFIER", password);
  //wifi_station_set_hostname(hostname);  
  Serial.println();
  Serial.println(WiFi.localIP());

  if(shouldSaveConfig && config.OTApassword != otapassword) {
    strcpy(otapassword, set_otapassword.getValue());
    Serial.print("New OTA password:");
    Serial.println(otapassword);
    UpdateConfig();
  }

  Serial.print("BME280 init");
  Wire.begin(D2,D5);  
  bmeStatus = bme.begin(0x76);
  windowStartTime = millis(); // Initialize start time
  if (!bmeStatus) {
    Serial.println(F("Could not find a valid BME280 sensor, check wiring!"));
    while (!bmeStatus && millis()-windowStartTime < 2500){ // retry 5x then giveup
      bmeStatus = bme.begin(0x76);
      delay(500);
      Serial.print(".");
   }
  }

  bme.setSampling(Adafruit_BME280::MODE_FORCED,
    Adafruit_BME280::SAMPLING_X1, // temperature
    Adafruit_BME280::SAMPLING_X1, // pressure
    Adafruit_BME280::SAMPLING_X16, // humidity
    Adafruit_BME280::FILTER_OFF);

  Serial.println("SPIFFS init");
  if(!SPIFFS.begin()){
    Serial.println(F("An Error has occurred while mounting SPIFFS"));
  }
  // Define RelayPin as output
  pinMode(RelayPin, OUTPUT); 

  Serial.println("PID init");
  myPID.SetSampleTime(WindowSize/10); //60s
  myPID.SetOutputLimits(0, WindowSize); 
  myPID.SetTunings(Kp,Ki,Kd,P_ON_M);
  myPID.SetMode(AUTOMATIC); 

  //http server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){ 
    request->send(SPIFFS, "/index.html");
  });
  // async data requests
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/plain", 
      (String(bmeRH) + ',' 
      + String((bmeTemp * 9/5)+32) + ',' // convert to Fahrenheit
      + String(Error) + ',' 
      + String(relayOn?"1":"0") + ',' 
      + String(constrain(Error/WindowSize*100,0,100)) + ',' //duty cycle
      + String((WindowSize-(millis()-windowStartTime))/1000) //pwm seconds
      ).c_str());
  });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
     request->send_P(200, "text/plain", String("{pid: ["+String(Kp)+","+String(Ki)+","+String(Kd)+"]}").c_str());
  });
  // updates from web client
  server.on("/sp", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam("val")) {
      inputMessage = request->getParam("val")->value();
      Setpoint = constrain(inputMessage.toDouble(),0,100);
      configChange = 3;  //update in 3 seconds
    }
    request->send(200, "text/html",String(Setpoint).c_str()); 
  });
  //debug set RH
  server.on("/rh", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String val;
    double newVal;
    if (request->hasParam("val")) {
      val = request->getParam("val")->value();
      newVal = val.toDouble();
      bmeRH = constrain(newVal,0,100);
    }
    request->send(200, "text/html",String("OK").c_str()); 
  });
  server.on("/pid", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String val;
    double newVal;
    if(request->hasParam("P")){
      val = request->getParam("P")->value();
      newVal = val.toDouble();
      Kp = constrain(newVal,0,1000);
      myPID.SetTunings(Kp,Ki,Kd,P_ON_M);
      configChange = 2; //update in 30 seconds
    }
    if(request->hasParam("I")){
      val = request->getParam("I")->value();
      newVal = val.toDouble();
      Ki = constrain(newVal,0,1000);
      myPID.SetTunings(Kp,Ki,Kd,P_ON_M);
      configChange = 2; //update in 30 seconds
    }
    if(request->hasParam("D")){
      val = request->getParam("D")->value();
      newVal = val.toDouble();
      Kd = constrain(newVal,0,1000);
      myPID.SetTunings(Kp,Ki,Kd,P_ON_M);
      configChange = 2; //update in 30 seconds
    }
    request->send_P(200, "text/plain", String("{"+String(Kp)+","+String(Ki)+","+String(Kd)+"}").c_str());
  });
  server.on("/window", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam("val")) {
      inputMessage = request->getParam("val")->value();
      int newWindow = inputMessage.toInt();
      WindowSize = constrain(newWindow,1,3600000);
      myPID.SetOutputLimits(0, WindowSize); 
      myPID.SetSampleTime(WindowSize/10);
      myPID.SetMode(AUTOMATIC);
      configChange = 2;  //update in 30 seconds
    }
    request->send(200, "text/html",String(WindowSize).c_str()); 
  });
  //powerstate
  server.on("/power", HTTP_GET, [] (AsyncWebServerRequest *request) {
    String inputMessage;
    if (request->hasParam("state")) {
      inputMessage = request->getParam("state")->value();
      powerOn = inputMessage=="1"?true:false; 
    }
    request->send(200, "text/html",String(powerOn?"1":"0").c_str()); 
  });
  server.onNotFound(notFound); //404
  //cache SPIFFS pages 1 day
  server.serveStatic("/",   SPIFFS, "/"  ,"max-age=86400"); 
  Serial.println("server start");
  server.begin();
}

void PrintValues(){
  Serial.print("PID:");
  Serial.print(Kp);Serial.print(",");Serial.print(Ki);Serial.print(",");Serial.print(Kd);
  Serial.print(" POW:");
  Serial.print(powerOn);
  Serial.print(" T:");
  Serial.print(bmeTemp);
  Serial.print(" H:");
  Serial.print(bmeRH);
  Serial.print(" SP:");
  Serial.print(Setpoint);
  Serial.print(" OUT:");
  Serial.print(Output/1000);
  Serial.print(" RELAY:");
  Serial.print(relayOn);
  Serial.print(" start:");
  Serial.print(windowStartTime/1000);
  Serial.print(" now:");
  Serial.print((millis()/1000));
  Serial.print(" diff:");
  Serial.print((millis()/1000 - windowStartTime/1000));
  Serial.print("       \r");
}

uint8_t slowloop;
void loop() {
  // Setting PID input to RH 
  Input = bmeRH;     
  // Call the PID compute function
  myPID.Compute();
  Error = Output*100;

   // 500ms loop
  delt_t = millis() - count;
  // update once per 500ms independent of read rate
  if (delt_t > 1000) {
		count = millis();
	  ArduinoOTA.handle();

    //serial debug
    //PrintValues();

    unsigned long now = millis();   
    //time to shift the PWM?
    if ((now - windowStartTime) > WindowSize) {
        windowStartTime += WindowSize;
    }
    ReadSensor();  
    // 15sec loop (minimum relay on/off time)
    slowloop++;
    if(slowloop>MIN_SWITCH_T){         
 
      // Turn the relay ON for the first part of the window and off for the rest
      relayOn = Error > (now - windowStartTime);      
      if(powerOn){     
        digitalWrite(RelayPin, relayOn);
      }
      else { 
        relayOn = false;
        digitalWrite(RelayPin,LOW);
      }

      //limit eeprom updates from web
      if(configChange>0){
        configChange--;
        if(configChange==0){UpdateConfig();}
      }
      slowloop=0;
    }
  }
  delay(1);
}
