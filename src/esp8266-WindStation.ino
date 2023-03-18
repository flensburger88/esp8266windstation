#include <FS.h> //this needs to be first, or it all crashes and burns...

#include <DHT.h> //https://github.com/adafruit/DHT-sensor-library
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Ticker.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>
#include <ESP8266httpUpdate.h>
#include <WiFiManager.h> //https://github.com/tzapu/WiFiManager
#include <DNSServer.h>
#include <ArduinoJson.h> //https://github.com/bblanchon/ArduinoJson
#include <TimeLib.h>

#define LEN_MQTTSERVER 60
#define LEN_MQTTPORT 6
#define LEN_MQTTUSER 40
#define LEN_MQTTPASS 40
#define LEN_KCWIND 4
#define LEN_WINDUGUUID 30
#define LEN_WINDGUTUPASS 20
#define LEN_WINDYKEY 128
#define LEN_VANEOFFSET 4
#define LEN_VANEMAXADC 5
#define LEN_WINDYAPP_SECRET 15 //?
#define LEN_WINDYAPP_ID 5      //?

char mqtt_server[LEN_MQTTSERVER];
char mqtt_port[LEN_MQTTPORT] = "1883";
char mqtt_user[LEN_MQTTUSER];
char mqtt_pass[LEN_MQTTPASS];
char kc_wind[LEN_KCWIND] = "0";
char windguru_uid[LEN_WINDUGUUID];
char windguru_pass[LEN_WINDGUTUPASS];
char windy_key[LEN_WINDYKEY];
char vaneOffset[LEN_VANEOFFSET] = "0";
char vaneMaxADC[LEN_WINDYAPP_ID] = "1023"; // ADC range for input voltage 0..1V
char windyAppSecret[LEN_WINDYAPP_SECRET] = "";
char windyAppID[LEN_WINDYAPP_ID] = "";

WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, LEN_MQTTSERVER);
WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, LEN_MQTTPORT);
WiFiManagerParameter custom_mqtt_user("user", "mqtt user", mqtt_user, LEN_MQTTUSER);
WiFiManagerParameter custom_mqtt_pass("pass", "mqtt password", mqtt_pass, LEN_MQTTPASS);
WiFiManagerParameter custom_kc_wind("kc_wind", "wind correction 1-999%", kc_wind, LEN_KCWIND);
WiFiManagerParameter custom_windguru_uid("windguru_uid", "windguru station UID", windguru_uid, LEN_WINDUGUUID);
WiFiManagerParameter custom_windguru_pass("windguru_pass", "windguru pass", windguru_pass, LEN_WINDGUTUPASS);
WiFiManagerParameter custom_windy_key("windy_key", "windy api key", windy_key, LEN_WINDYKEY);
WiFiManagerParameter custom_vaneMaxADC("vaneMaxADC", "Max ADC value 1-1024", vaneMaxADC, LEN_VANEMAXADC);
WiFiManagerParameter custom_vaneOffset("vaneOffset", "Wind vane offset 0-359", vaneOffset, LEN_VANEOFFSET);
WiFiManagerParameter custom_windyAppId("windyAppID", "WindyApp user Id", windyAppID, LEN_WINDYAPP_ID);
WiFiManagerParameter custom_windyAppSecret("windyAppSecret", "Windy App secret", windyAppSecret, LEN_WINDYAPP_SECRET);

String st;
String content;
int statusCode;

int debouncing_time = 10000; // time in microseconds!
unsigned long last_micros = 0;
volatile int windimpulse = 0;

#define VERSION "v1.92 OTA"
#define VERSIONINFO "\n\n----------------- GAYIK Wind Station v1.92 OTA -----------------"
#define NameAP "WindStationAP"

#define OTA_CHECK_MINUTES 30

// #define FirmwareURL "http://gayikweatherstation.blob.core.windows.net/firmware/esp8266-WindStation.ino.generic.bin" // URL of firmware file for http OTA update by secret MQTT command "flash"
#define FirmwareURL ""
#define MQTT_CONNECT_RETRIES 5

// #define NightSleepMODE                                       // Enable deep-sleep only in night time
#if defined(DeepSleepMODE) || defined(NightSleepMODE) // Deep-sleep mode power consumption ~6mAh (3*5=15sec work/5 min sleep), instead ~80mAh in default "Always On" mode

#define SLEEPNIGHT 10
#define TIMEZONE 2        // UTC offset
#define TIMEMORNING 5     // night end at...
#define TIMEEVENING 20    // night start at...
#include "NtpClientLib.h" //https://github.com/gmag11/NtpClient
#endif

// #define MOSFETPIN       15                                   // Experemental!!! GPIO15 (D8 for NodeMcu). MosFET's gate pin for power supply sensors, off for current drain minimize. Not connect this GPIO directly to sensors you burning it! The maximum source current of GPIO is about 12mA

#define BUTTON 4   // optional, GPIO4 for Witty Cloud. GPIO0/D3 for NodeMcu (Flash) - not work with deepsleep, set 4!
#define LED 2      // GPIO2 for Witty Cloud. GPIO16/D0 for NodeMcu - not work with deepsleep, set 2!
#define DHTPIN D5  // GPIO14 (D5 for NodeMcu)
#define WINDPIN D1 // GPIO5 (D1 for NodeMcu)

#define MQTT_TOPIC "windpoint" // mqtt topic (Must be unique for each device)
#define MQTT_TOPICm "windpoint/m"
#define MQTT_TOPICo "windpoint/o"

#ifdef DHTPIN
#define DHTTYPE DHT22     // DHT11, DHT22, DHT21, AM2301
DHT dht(DHTPIN, DHTTYPE); //
#endif

bool sendStatus = true;
bool sensorReport = false;
bool resetWind = true;
bool firstRun = true;

int errors_count = 0;
int kUpdFreq = 1; // minutes
bool firstWindReceived = 0;

const float kKnots = 1.94;     // m/s to knots conversion
#define windMeasurePeriodSec 3 // wind measurement period in seconds 1-10sec / 3!

float dhtH, dhtT, windMS = 0;
float WindMax = 0, WindAvr = 0, WindMin = 100;

String str;

unsigned long TTasks;
unsigned long secTTasks;
unsigned long count_btn = 0;

float windSpeed = 0; // Wind speed (mph)

volatile unsigned long Rotations;         // cup rotation counter used in interrupt routine
volatile unsigned long ContactBounceTime; // Timer to avoid contact bounce in interrupt routine
int vane_value;                           // raw analog value from wind vane
int Direction;                            // translated 0 - 360 direction
int CalDirection;                         // converted value with offset applied

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
#define MSG_BUFFER_SIZE (50)
char msg_buff[MSG_BUFFER_SIZE];

Ticker btn_timer;

// config for rest service
WiFiManager wifiManager;

//--------------------------------START OF CODE SECTION--------------------------------------------------------
unsigned long startTime = 0;
//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////Get wind speed  /////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void getWindSpeed(void)
{
  if (startTime == 0)
  {
    Rotations = 0; // Set Rotations count to 0 ready for calculations
    startTime = millis();
  }

  long duration = millis() - startTime;
  if (duration > (windMeasurePeriodSec * 1000)) // 3 Seconds after the start of the measurement
  {
    /* convert to mp/h using the formula V=P(2.25/T)
     V = P(2.25/3) = P * 0.75       V - speed in mph,  P - pulses per sample period, T - sample period in seconds
     V = P(2.25/(duration/1000))  -> V = P * (2,25 * 1000) / duration -> P * 2250 / duration
      */
    // windSpeed = Rotations * 0.75; // 3 seconds
    windSpeed = Rotations * 2250 / duration; // duration millis
    Rotations = 0;                           // Reset count for next sample

    if (windSpeed > WindMax)
    {
      WindMax = windSpeed;
    }
    if (WindMin > windSpeed)
    {
      WindMin = windSpeed;
    }

    WindAvr = (WindMax + WindMin) * 0.5; // average wind speed mph per 10 minutes

    firstWindReceived = true;

    Rotations = 0; // Set Rotations count to 0 ready for calculations
    startTime = millis();
    Serial.print("found wind (mp/h): ");
    Serial.print(windSpeed);
    Serial.print("\tendured (ms): ");
    Serial.println(duration);
  }
}

// This is the function that the interrupt calls to increment the rotation count
//-------------------------------------------------------------------------------------------------------------
////////////////////////////////////ISR rotation//////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void IRAM_ATTR isr_rotation()
{
  if ((millis() - ContactBounceTime) > 15)
  { // debounce the switch contact.
    Rotations++;
    ContactBounceTime = millis();
  }
}
// Get Wind Direction
//-------------------------------------------------------------------------------------------------------------
/////////////////////////////////// Wind direction ////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void getWindDirection(void)
{
  vane_value = analogRead(A0);                              // read sensor data
  Direction = map(vane_value, 0, atoi(vaneMaxADC), 0, 359); // map it to 360 degrees
  CalDirection = Direction + atoi(vaneOffset);              // factor in the offset degrees

  // Fix degrees:
  if (CalDirection > 360)
    CalDirection = CalDirection - 360;

  if (CalDirection < 0)
    CalDirection = CalDirection + 360;
}

// Convert MPH to Knots
float getKnots(float speed)
{
  return speed * 0.868976; // knots 0.868976;
}
// Convert MPH to m/s
float getms(float speed)
{
  return speed * 0.44704; // metric m/s 0.44704;;
}

//-------------------------------------------------------------------------------------------------------------
/////////////////////////////////// MQTT Received message callback ////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
void callback(char *topic, byte *payload, unsigned int length)
{
  String payload_string = String((char *)payload);
  payload_string = payload_string.substring(0, length);
  Serial.println("MQTT Topic: " + String(topic) + " MQTT Payload: " + payload_string);
  if (payload_string == "stat")
  {
  }
  else if (payload_string == "reset")
  {
    errors_count = 100;
  }
  else if (payload_string == "sensor")
  {
    if (minute() < 10)
      mqttClient.publish("debug", (String(hour()) + ":0" + String(minute())).c_str());
    else
      mqttClient.publish("debug", (String(hour()) + ":" + String(minute())).c_str());
    sensorReport = true;
  }
  else if (payload_string == "adc")
  {
    mqttClient.publish("debug", ("ADC:" + String(analogRead(A0)) + " error:" + String(errors_count)).c_str());
  }
  else if (payload_string == "flash")
  {
    flashOTA(true);
  }
  else
  {
    // We check the topic in order to see what kind of payload we got:
    str = payload_string;
    int i = atoi(str.c_str());
    if ((i >= 0) && (i < 9999))
    {
      if (String(topic) == MQTT_TOPIC) // we got kc_wind?
        strcpy(kc_wind, String(i).c_str());
      else if (String(topic) == MQTT_TOPICm) // we got vaneMaxADC?
        strcpy(vaneMaxADC, String(i).c_str());
      else if (String(topic) == MQTT_TOPICo) // we got vaneOffset?
        strcpy(vaneOffset, String(i).c_str());

      mqttClient.publish("debug", "saving config");
      Serial.println("saving config");
      DynamicJsonDocument json(1024);
      json["mqtt_server"] = mqtt_server;
      json["mqtt_port"] = mqtt_port;
      json["mqtt_user"] = mqtt_user;
      json["mqtt_pass"] = mqtt_pass;
      json["kc_wind"] = kc_wind;
      json["windguru_uid"] = windguru_uid;
      json["windguru_pass"] = windguru_pass;
      json["windy_key"] = windy_key;
      json["vaneMaxADC"] = vaneMaxADC;
      json["vaneOffset"] = vaneOffset;

      File configFile = SPIFFS.open("/config.json", "w");
      if (!configFile)
      {
        Serial.println("failed to open config file for writing");
      }

      serializeJson(json, Serial);
      serializeJson(json, configFile);
      configFile.close();

      sendStatus = true;
    }
  }
}

// callback notifying us of the need to save config
void saveConfigCallback()
{
  Serial.println("Should save config");

  // read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(mqtt_user, custom_mqtt_user.getValue());
  strcpy(mqtt_pass, custom_mqtt_pass.getValue());
  strcpy(kc_wind, custom_kc_wind.getValue());
  strcpy(windguru_uid, custom_windguru_uid.getValue());
  strcpy(windguru_pass, custom_windguru_pass.getValue());
  strcpy(windy_key, custom_windy_key.getValue());
  strcpy(vaneMaxADC, custom_vaneMaxADC.getValue());
  strcpy(vaneOffset, custom_vaneOffset.getValue());
  strcpy(windyAppID, custom_windyAppId.getValue());
  strcpy(windyAppSecret, custom_windyAppSecret.getValue());

  // save the custom parameters to FS

  Serial.println("saving config");
  DynamicJsonDocument json(1024);
  json["mqtt_server"] = mqtt_server;
  json["mqtt_port"] = mqtt_port;
  json["mqtt_user"] = mqtt_user;
  json["mqtt_pass"] = mqtt_pass;
  json["kc_wind"] = kc_wind;
  json["windguru_uid"] = windguru_uid;
  json["windguru_pass"] = windguru_pass;
  json["windy_key"] = windy_key;
  json["vaneMaxADC"] = vaneMaxADC;
  json["vaneOffset"] = vaneOffset;
  json["windyAppID"] = windyAppID;
  json["windyAppSecret"] = windyAppSecret;

  File configFile = SPIFFS.open("/config.json", "w");
  if (!configFile)
  {
    Serial.println("failed to open config file for writing");
  }

  serializeJson(json, Serial);
  serializeJson(json, configFile);
  configFile.close();
  // end save parameters
  ESP.restart();
}

unsigned long nextOTUpdate = 0;

void flashOTA(bool force)
{
  if (nextOTUpdate == 0)
  {
    nextOTUpdate = millis() + (OTA_CHECK_MINUTES * 1000);
  }

  if (!force && millis() < nextOTUpdate)
  {
    return;
  }
  nextOTUpdate = 0;

  // disabling the ota update
  if (String(FirmwareURL).length() == 0)
  {
    Serial.println("OTA Update disabled");
    return;
  }

  Serial.println("HTTP_UPDATE FILE: " + String(FirmwareURL));
  // noInterrupts();
  detachInterrupt(digitalPinToInterrupt(WINDPIN));
  WiFiClient client;

  ESPhttpUpdate.setLedPin(LED, LOW);
  t_httpUpdate_return ret = ESPhttpUpdate.update(client, FirmwareURL);
  attachInterrupt(WINDPIN, isr_rotation, FALLING);

  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
    ESP.restart();
    break;
  case HTTP_UPDATE_NO_UPDATES:
    Serial.println("HTTP_UPDATE_NO_UPDATES");
    break;
  case HTTP_UPDATE_OK:
    Serial.println("HTTP_UPDATE_OK");
    break;
  }
}

void setupSpiffs()
{
  // read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin())
  {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json"))
    {
      // file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");

        DynamicJsonDocument json(1024);
        DeserializationError error = deserializeJson(json, configFile);
        if (error)
        {
          Serial.print(F("deserializeJson() failed with code "));
          Serial.println(error.c_str());
        }
        else
        {
          serializeJson(json, Serial);
          Serial.println("\nparsed json");
          strcpy(mqtt_server, json["mqtt_server"] | mqtt_server);
          strcpy(mqtt_port, json["mqtt_port"] | mqtt_port);
          strcpy(mqtt_user, json["mqtt_user"] | mqtt_user);
          strcpy(mqtt_pass, json["mqtt_pass"] | mqtt_pass);
          strcpy(kc_wind, json["kc_wind"] | kc_wind);
          strcpy(windguru_uid, json["windguru_uid"] | windguru_uid);
          strcpy(windguru_pass, json["windguru_pass"] | windguru_pass);
          strcpy(windy_key, json["windy_key"] | windy_key);
          strcpy(vaneMaxADC, json["vaneMaxADC"] | vaneMaxADC);
          strcpy(vaneOffset, json["vaneOffset"] | vaneOffset);
          strcpy(windyAppID, json["windyAppID"] | windyAppID);
          strcpy(windyAppSecret, json["windyAppSecret"] | windyAppSecret);
          Serial.println("\nAfter Reading");

          custom_mqtt_server.setValue(mqtt_server, LEN_MQTTSERVER);
          custom_mqtt_port.setValue(mqtt_port, LEN_MQTTPORT);
          custom_mqtt_user.setValue(mqtt_user, LEN_MQTTUSER);
          custom_mqtt_pass.setValue(mqtt_pass, LEN_MQTTPASS);
          custom_kc_wind.setValue(kc_wind, LEN_KCWIND);
          custom_windguru_uid.setValue(windguru_uid, LEN_WINDUGUUID);
          custom_windguru_pass.setValue(windguru_pass, LEN_WINDGUTUPASS);
          custom_windy_key.setValue(windy_key, LEN_WINDYKEY);
          custom_vaneMaxADC.setValue(vaneMaxADC, LEN_VANEMAXADC);
          custom_vaneOffset.setValue(vaneOffset, LEN_VANEOFFSET);
          custom_windyAppId.setValue(windyAppID, LEN_WINDYAPP_ID);
          custom_windyAppSecret.setValue(windyAppSecret, LEN_WINDYAPP_SECRET);
        }
      }
    }
  }
  else
  {
    Serial.println("failed to mount FS");
  }
  // end read configuration
}
void connectMQTT()
{

  if (String(mqtt_server).length() == 0)
  {
    Serial.println("Mqtt Broker not configured");
  }
  else
  {
    short kRetries = MQTT_CONNECT_RETRIES;
    Serial.print("Connecting to ");
    Serial.print(mqtt_server);
    Serial.print(" Broker . .");
    delay(500);
    mqttClient.setServer(mqtt_server, atoi(mqtt_port));
    mqttClient.setSocketTimeout(70);
    mqttClient.setKeepAlive(70);
    while (!mqttClient.connected() && kRetries--)
    {
      String str = String(ESP.getChipId(), HEX);
      mqttClient.connect(str.c_str(), mqtt_user, mqtt_pass);
      Serial.print(" .");
      delay(1000);
    }
    if (mqttClient.connected())
    {
      Serial.println(" DONE");
      Serial.println("\n----------------------------  Logs  ----------------------------");
      Serial.println();
      mqttClient.subscribe(MQTT_TOPIC);
      mqttClient.subscribe(MQTT_TOPICm);
      mqttClient.subscribe(MQTT_TOPICo);
      blinkLED(LED, 40, 8);
      digitalWrite(LED, LOW);
      mqttClient.publish("start", ("Version:" + String(VERSION)).c_str());
    }
    else
    {
      Serial.println(" FAILED!");
      Serial.println("\n----------------------------------------------------------------");
      Serial.println();
      digitalWrite(LED, HIGH);
    }
  }
}

void setup()
{
  Serial.begin(9600);
  Serial.println(VERSIONINFO);
  Serial.print("\nESP ChipID: ");
  Serial.println(ESP.getChipId(), HEX);

  // clean FS, erase config.json in case of damaged file
  // SPIFFS.format();

  Rotations = 0; // Set Rotations to 0 ready for calculations

  setupSpiffs();

  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length

#ifdef MOSFETPIN
  pinMode(MOSFETPIN, OUTPUT);
  digitalWrite(MOSFETPIN, HIGH);
#endif

  pinMode(LED, OUTPUT);
  pinMode(BUTTON, INPUT);
  pinMode(WINDPIN, INPUT_PULLUP);
  digitalWrite(LED, HIGH);

  firstRun = true;
  btn_timer.attach(0.05, button);

  mqttClient.setCallback(callback);

  wifiManager.setSaveConfigCallback(saveConfigCallback); // set config save notify callback

#ifdef DeepSleepMODE
  wifiManager.setTimeout(60); // sets timeout until configuration portal gets turned off
#else
  wifiManager.setTimeout(180); // sets timeout until configuration portal gets turned off
#endif

  // add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_mqtt_user);
  wifiManager.addParameter(&custom_mqtt_pass);
  wifiManager.addParameter(&custom_kc_wind);
  wifiManager.addParameter(&custom_windguru_uid);
  wifiManager.addParameter(&custom_windguru_pass);
  wifiManager.addParameter(&custom_windy_key);
  wifiManager.addParameter(&custom_vaneMaxADC);
  wifiManager.addParameter(&custom_vaneOffset);
  wifiManager.addParameter(&custom_windyAppId);
  wifiManager.addParameter(&custom_windyAppSecret);

  WiFi.mode(WIFI_STA); // explicitly set mode, esp defaults to STA+AP
  WiFi.setSleepMode(WIFI_NONE_SLEEP);

  if (!wifiManager.autoConnect(NameAP))
  {
    Serial.println("failed to connect and hit timeout");
    delay(1000);
    // reset and try again, or maybe put it to deep sleep

#if defined(DeepSleepMODE) || defined(NightSleepMODE)
    ESP.deepSleep(SLEEPNIGHT * 60000000); // Sleep for x* minute(s)
#endif
  }

  Serial.print("\nChecking Wifi Connection");
  if ((WiFi.status() != WL_CONNECTED))
  {
    Serial.println(" Not Connected to Wifi - Restarting");
    ESP.restart();
    delay(5000);
    return;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
#if defined(DeepSleepMODE) || defined(NightSleepMODE)
    NTP.begin("pool.ntp.org", TIMEZONE, true); // ntpServerName, NTP_TIMEZONE, daylight = false
    NTP.setInterval(10, 600);                  // ShortInterval, LongInterval
#endif
    Serial.println(" DONE");
    Serial.print("IP Address is: ");
    Serial.println(WiFi.localIP());
    Serial.print("macAddress is: ");
    Serial.println(WiFi.macAddress());
    connectMQTT();
  }

  // TODO: Start Webserver to modify parameters in runtime
  wifiManager.stopConfigPortal();
  wifiManager.startWebPortal();

  attachInterrupt(WINDPIN, isr_rotation, FALLING);
}

void loop()
{
  wifiManager.process();

  flashOTA(false);
  mqttClient.loop();
  timedTasks();
  checkStatus();
  if (sensorReport)
  {
    getSensors();
  }
  delay(50);
}

void blinkLED(int pin, int duration, int n)
{
  for (int i = 0; i < n; i++)
  {
    digitalWrite(pin, HIGH);
    delay(duration);
    digitalWrite(pin, LOW);
    delay(duration);
  }
}

void button()
{
  if (!digitalRead(BUTTON))
  {
    count_btn++;
  }
  else
  {
    if (count_btn > 1 && count_btn <= 40)
    {
      digitalWrite(LED, !digitalRead(LED));
      sendStatus = true;
    }
    else if (count_btn > 40)
    {
      Serial.println("\n\nESP8266 Rebooting . . . . . . . . Please Wait");
      errors_count = 100;
    }
    count_btn = 0;
  }
}

void checkMQTTConnection()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    errors_count = errors_count + 10;
    Serial.println("WiFi connection . . . LOST errors_count = " + String(errors_count));
  }
  else if (String(mqtt_server).length() == 0)
  {
    Serial.println("mqtt broker connection . . . ..not configured");
  }
  else if (mqttClient.connected())
  {
    Serial.println("mqtt broker connection . . . . . . . . . . OK");
  }
  else
  {
    errors_count = errors_count + 5;
    Serial.println("mqtt broker connection. . . . LOST errors_count = " + String(errors_count));
  }
}

void checkStatus()
{
  if (sendStatus)
  {
    mqttClient.publish("kc_wind", String(kc_wind).c_str());
    mqttClient.publish("adc", ("{\"ADC\":" + String(analogRead(A0)) + ", " + "\"MaxADC\":" + String(vaneMaxADC) + ", " + "\"Offset\":" + String(vaneOffset) + "}").c_str());
    sendStatus = false;
  }
  if (errors_count >= 100)
  {
    blinkLED(LED, 400, 4);
    ESP.restart();
    delay(500);
  }
}

void getSensors()
{
  String pubString;
#ifdef DHTPIN
  Serial.print("DHT read . . . . . . . . . . . . . . . . . ");
  dhtH = dht.readHumidity();
  dhtT = dht.readTemperature();
  if (digitalRead(LED) == LOW)
  {
    blinkLED(LED, 100, 1);
  }
  else
  {
    blinkLED(LED, 100, 1);
    digitalWrite(LED, HIGH);
  }
  if (isnan(dhtH) || isnan(dhtT))
  {
    if (mqttClient.connected())
      mqttClient.publish("debug", "DHT READ ERROR");
    Serial.println("ERROR");
  }
  else
  {
    pubString = "{\"Temp\": " + String(dhtT) + ", " + "\"Humidity\": " + String(dhtH) + "}";
    pubString.toCharArray(msg_buff, pubString.length() + 1);
    if (mqttClient.connected())
      mqttClient.publish("temp", msg_buff);
    Serial.println("OK");
  }
#endif

#ifdef DeepSleepMODE
  if (mqttClient.connected())
    mqttClient.publish("debug", "ADC:" + String(a0) + " " + NTP.getTimeDateString());
#endif

  if (firstWindReceived)
  { // already made measurement wind power
    pubString = "{\"Min\": " + String(WindMin, 2) + ", " + "\"Avr\": " + String(WindAvr) + ", " + "\"Max\": " + String(WindMax, 2) + ", " + "\"Dir\": " + String(CalDirection) + "}";
    pubString.toCharArray(msg_buff, pubString.length() + 1);
    if (mqttClient.connected())
      mqttClient.publish("wind", msg_buff);
    Serial.print(" Wind Min: " + String(getKnots(WindMin), 2) + " Avr: " + String(getKnots(WindAvr), 2) + " Max: " + String(getKnots(WindMax), 2) + " Dir: " + String(CalDirection) + " (offset:" + String(vaneOffset) + ") ");
  }
  resetWind = true;
  sensorReport = false;
}

void timedTasks()
{
  getWindSpeed();
  getWindDirection();

  // kUpdFreq minutes timer
  if ((millis() > TTasks + (kUpdFreq * 60000)) || (millis() < TTasks))
  {
    if ((WindMax > 2) && (WindMin == WindMax))
      errors_count = errors_count + 25; // check for freeze CPU
    TTasks = millis();
    checkMQTTConnection();
    // sensorReport = true;
    getSensors();
    SendData();
  }
}

void SendData()
{

  if (!SendToWindguru())
    errors_count++;

  if (!SendToWindyCom())
    errors_count++;

  if (!SendToWindyApp())
    errors_count++;

  ResetCounters();
}

void ResetCounters()
{
  WindMax = 0;
  WindMin = 100;
}

//-------------------------------------------------------------------------------------------------------------
/////////////////////////////////// SEND TO WINDGURU //////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
bool SendToWindguru()
{ // send info to windguru.cz
  WiFiClient client;
  HTTPClient http; // must be declared after WiFiClient for correct destruction order, because used by http.begin(client,...)
  String getData = "", Link;
  unsigned long time;

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("wi-fi connection failed");
    return false; // fail;
  }
  else if (String(windguru_uid).length() == 0)
  {
    Serial.println("\nwindguru uid not configured");
    return true; // not an failure / just not configured;
  }
  else
  { // Check WiFi connection status
    Link = "http://www.windguru.cz/upload/api.php?";
    time = millis();

    //--------------------------md5------------------------------------------------
    MD5Builder md5;
    md5.begin();
    md5.add(String(time) + String(windguru_uid) + String(windguru_pass));
    md5.calculate();
    if (firstWindReceived)
      getData = "uid=" + String(windguru_uid) + "&salt=" + String(time) + "&hash=" + md5.toString() + "&interval=" + String(kUpdFreq) + "&wind_min=" + String(getKnots(WindMin), 2) + "&wind_avg=" + String(getKnots(WindAvr), 2) + "&wind_max=" + String(getKnots(WindMax), 2);
    // wind_direction     wind direction as degrees (0 = north, 90 east etc...)
    getData = getData + "&wind_direction=" + String(CalDirection);
#ifdef DHTPIN
    if (!isnan(dhtT))
      getData = getData + "&temperature=" + String(dhtT);
    if (!isnan(dhtH))
      getData = getData + "&rh=" + String(dhtH);
#endif
    Serial.println(Link + getData);
    http.begin(client, Link + getData); // Specify request destination
    int httpCode = http.GET();          // Send the request
    if (httpCode > 0)
    {                                    // Check the returning code
      String payload = http.getString(); // Get the request response payload
      Serial.println(payload);           // Print the response payload
      if (mqttClient.connected() && (payload != "OK"))
        mqttClient.publish("debug", payload.c_str());
    }
    http.end(); // Close connection
  }

  return true; // done
}

//-------------------------------------------------------------------------------------------------------------
//////////////////////////////////// SEND TO WINDY ////////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
// https://stations.windy.com/pws/update/XXX-API-KEY-XXX?winddir=230&windspeedmph=12&windgustmph=12&tempf=70&rainin=0&baromin=29.1&dewptf=68.2&humidity=90
// We will be displaying data in 5 minutes steps. So, it's not nessary send us data every minute, 5 minutes will be fine.
// https://community.windy.com/topic/8168/report-your-weather-station-data-to-windy
bool SendToWindyCom()
{ // send info to http://stations.windy.com/stations
  WiFiClient client;
  HTTPClient http; // must be declared after WiFiClient for correct destruction order, because used by http.begin(client,...)
  String getData, Link;

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("wi-fi disconnected - not windy uploading");
    return false; // fail;
  }
  else if (String(windy_key).isEmpty())
  {
    Serial.println("windy not configured - skipping");
  }
  else
  { // Check WiFi connection status
    Link = "http://stations.windy.com/pws/update/" + String(windy_key) + "?name=windsurf&";

    // wind speed during interval (knots)
    if (firstWindReceived)
      getData = "winddir=" + String(CalDirection) + "&wind=" + String(WindAvr) + "&gust=" + String(WindMax, 2);

#ifdef DHTPIN
    if (!isnan(dhtT))
      getData = getData + "&temp=" + String(dhtT);
    if (!isnan(dhtH))
      getData = getData + "&rh=" + String(dhtH);
#endif
    Serial.println(Link + getData);
    http.begin(client, Link + getData); // Specify request destination
    int httpCode = http.GET();          // Send the request
    if (httpCode > 0)
    {                                    // Check the returning code
      String payload = http.getString(); // Get the request response payload
      Serial.println(payload);           // Print the response payload
      if (mqttClient.connected() && (payload.indexOf("SUCCESS") == -1))
        mqttClient.publish("debug", payload.c_str());
    }
    http.end(); // Close connection
  }

  return true; // done
}

//-------------------------------------------------------------------------------------------------------------
//////////////////////////////////// SEND TO WINDYAPP /////////////////////////////////////////////////////////
//-------------------------------------------------------------------------------------------------------------
// https://windyapp.co/apiV9.php?method=addCustomMeteostation&secret=WindyAPPSecret&d5=123&a=11&m=10&g=15&i=test1
// d5* - direction from 0 to 1024. direction in degrees is equal = (d5/1024)*360
// a* - average wind per sending interval. for m/c - divide by 10
// m* - minimal wind per sending interval. for m/c - divide by 10
// g* - maximum wind per sending interval. for m/c - divide by 10
// i* - device number

bool SendToWindyApp()
{                                   // send info to http://windy.app/
  const char *host = "windyapp.co"; // only google.com not https://google.com
  String getData = "", Link;

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("wi-fi connection failed");
    return false; // fail;
  }
  else if (String(windyAppID).isEmpty())
  {
    Serial.println("WindyApp not configured - skipping upload");
  }
  else
  { // Check WiFi connection status
    Link = "/apiV9.php?method=addCustomMeteostation&secret=" + String(windyAppSecret) + "&i=" + String(windyAppID) + "&";
    if (firstWindReceived)
      getData = "d5=" + String(map(CalDirection, 0, 359, 1, 1024)) + "&m=" + String(WindMin * 10, 0) + "&a=" + String(WindAvr * 10, 0) + "&g=" + String(WindMax * 10, 0);

#ifdef DHTPIN
    if (!isnan(dhtT))
      getData = getData + "&t2=" + String(dhtT);
    if (!isnan(dhtH))
      getData = getData + "&h=" + String(dhtH);
#endif
    getData.replace(" ", "");

    // Use WiFiClient class to create TCP connections
    WiFiClientSecure httpsClient;
    const int httpPort = 443;  // 80 is for HTTP / 443 is for HTTPS!
    httpsClient.setInsecure(); // this is the magical line that makes everything work
    if (!httpsClient.connect(host, httpPort))
    { // works!
      Serial.println("https connection failed");
      if (mqttClient.connected())
        mqttClient.publish("debug", "windyapp.co https connection failed");
      return false;
    }

    // We now create a URI for the request
    String url = Link + getData;

    // This will send the request to the server
    Serial.println("request sent: " + url);
    httpsClient.print(String("GET ") + url + " HTTP/1.1\r\n" +
                      "Host: " + host + "\r\n" +
                      "Connection: close\r\n\r\n");

    while (httpsClient.connected())
    {
      String line = httpsClient.readStringUntil('\n');
      if (line == "\r")
      {
        Serial.println("headers received");
        break;
      }
    }

    String payload;
    while (httpsClient.available())
    {
      payload = httpsClient.readString(); // Read Line by Line
    }
    payload.replace("\n", "/");
    Serial.println("reply was: " + payload); // Print response
    if (mqttClient.connected() && (payload.indexOf("success") == -1))
    {
      payload = getData + "> " + payload;
      mqttClient.publish("debug", payload.c_str());
    }
  }

  return true; // done
}
