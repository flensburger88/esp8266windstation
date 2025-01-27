# esp8266-WindStation Revised Edition

## Simple wind station for ESP8266 based circuits

This project is based on the original esp8266-WindStation project located at:
https://github.com/zpukr/esp8266-WindStation
and for ESP32: https://github.com/zpukr/esp32-WindStation <br>

Changes to the original code (by emreetugrul):

- code cleanup, refactoring & commenting, simpler flow
- improved functionality for reading wind speed and direction (for Davis 6410)
- improved OTA
- improved configuration management over MQTT

Additional Changes (by Flensburger88)

- migrated to PIO / Removed Offline Code Copies
- Cleaned up the code
- (temporarily) disabled the ota update mechanism
- optimized the error handling messages
- its not an error, if no mqtt is configured
- its not an error, if no windguru uid is configured
- its not an error, if no windyapp uid is configured
- its not an error, if no windy uid is configured
- enable configuration possibility after wifi is connected ( lazy portal )
- enable configuration for windyApp @runtime
- better reading logik for wind values / non blocking for Webserver

Todos:
- wind direction not read out correctly
- check field length for windyApp
- remove deprecated spiffs
- (!) check windyCom api
- (!) check windyApp api
- (!) check windGuru api
- (?) DHT22 not delivering data - error on read

- add custom weather server url
- add dynamic update url / configurable via setup routine
- docu for mqtt commands
- flexible send intervall ( via Settings? )


## Original Doku

An example of building a weather station on the ESP8266 Wemos D1 mini board (cost ~$4), Davis Anemometer ( ~$120) and DHT11 ( ~$1):
![alt tag](https://github.com/zpukr/esp8266-WindStation/blob/master/windstation.jpg)

Installation of the humidity/temperature sensor DHT-11 is option. You can set DHT-22 ( ~3$) or DHT-21 ( ~4$) instead it, which has a lot more accuracy and can show negative temperatures. Also, to minimize final costs, instead of the Davis Anemometer, you can use the cheap La Crosse TX23U sensor ( ~$50). Or even build an anemometer yourself from old computer fan, example on russian http://skootsone.yolasite.com/wind-pow-02.php

Flash a program to the ESP8266 with Arduino IDE. After first run ESP start as Access Point mode with SSID "WindStationAP" and default password "87654321", spins up a DNS and WebServer (default ip 192.168.4.1). Using any wifi enabled device with a browser (computer, phone, tablet) connect to the newly created Access Point. Set some parameters and click "Save":
![alt tag](https://github.com/zpukr/esp8266-WindStation/blob/master/WindStationAP.jpg)

After this ESP will try connect to internet. If successful, settings save to flash memory. If not, all settings are reset and WindStation start as Access Point again

Optionally, the station supports Deep Sleep mode for cases when only power is available from batteries or solar panels. In this mode, the consumption of approximately 6mAh (15 sec active/ 5min sleep) whereas in normal mode 80mAh

You can use MQTT control panel for real time viewing/adjust parameters and variables of weather station. Below ready-to-use template for the MQTT Dash application https://play.google.com/store/apps/details?id=net.routix.mqttdash&hl=uk

<details>
<summary>Click to expand</summary>
	[{"mainTextSize":"LARGE","postfix":" m/s","prefix":"","textColor":-192,"enableIntermediateState":true,"enablePub":false,"enteredIntermediateStateAt":0,"intermediateStateTimeout":10,"jsOnReceive":"","jsonPath":"$.Avr","lastJsonPathValue":"0.0","lastPayload":"{\"Min\": 0.00, \"Avr\": 0.00, \"Max\": 0.00, \"Dir\": 270}","qos":0,"retained":false,"topic":"windpoint/wind","topicPub":"windpoint","updateLastPayloadOnPub":false,"id":"9b0b155b-29c1-48fa-9085-7c54b0f73bbe","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1552427633,"longId":5,"name":"WindAvr","type":1},{"decimalPrecision":0,"displayPayloadValue":true,"maxValue":360.0,"minValue":0.0,"postfix":"°","prefix":"","progressColor":-192,"enableIntermediateState":true,"enablePub":false,"enteredIntermediateStateAt":0,"intermediateStateTimeout":1,"jsOnReceive":"","jsonPath":"$.Dir","lastJsonPathValue":"270","lastPayload":"{\"Min\": 0.00, \"Avr\": 0.00, \"Max\": 0.00, \"Dir\": 270}","qos":1,"retained":false,"topic":"windpoint/wind","topicPub":"windpoint","updateLastPayloadOnPub":false,"id":"716bae44-f2f5-4ce7-9f79-61292c0d2f2f","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1552427633,"longId":13,"name":"Direction","type":3},{"mainTextSize":"MEDIUM","postfix":" m/s","prefix":"","textColor":-12550144,"enableIntermediateState":true,"enablePub":false,"enteredIntermediateStateAt":0,"intermediateStateTimeout":0,"jsOnReceive":"","jsonPath":"$.Min","lastJsonPathValue":"0.0","lastPayload":"{\"Min\": 0.00, \"Avr\": 0.00, \"Max\": 0.00, \"Dir\": 270}","qos":0,"retained":false,"topic":"windpoint/wind","topicPub":"sensor","updateLastPayloadOnPub":false,"id":"fd6e2415-bad0-404c-a43b-b164171b8017","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1552427633,"longId":4,"name":"WindMin","type":1},{"mainTextSize":"MEDIUM","postfix":" m/s","prefix":"","textColor":-65472,"enableIntermediateState":true,"enablePub":false,"enteredIntermediateStateAt":0,"intermediateStateTimeout":0,"jsOnReceive":"","jsonPath":"$.Max","lastJsonPathValue":"0.0","lastPayload":"{\"Min\": 0.00, \"Avr\": 0.00, \"Max\": 0.00, \"Dir\": 270}","qos":0,"retained":false,"topic":"windpoint/wind","topicPub":"sensor","updateLastPayloadOnPub":false,"id":"7eed0deb-cb11-4ec5-ab1f-9ce9305fb341","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1552427633,"longId":6,"name":"WindMax","type":1},{"decimalPrecision":0,"displayPayloadValue":true,"maxValue":140.0,"minValue":1.0,"postfix":"","prefix":"","progressColor":-1,"enableIntermediateState":false,"enablePub":true,"enteredIntermediateStateAt":0,"intermediateStateTimeout":10,"jsOnReceive":"","jsonPath":"","lastPayload":"15","qos":1,"retained":false,"topic":"windpoint/kc_wind","topicPub":"windpoint","updateLastPayloadOnPub":false,"id":"4d50b3f6-8f21-40b9-b90b-17031b769896","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1552427672,"longId":8,"name":"kcWind","type":3},{"decimalPrecision":0,"displayPayloadValue":true,"maxValue":1023.0,"minValue":1.0,"postfix":"","prefix":"","progressColor":-1,"enableIntermediateState":false,"enablePub":true,"enteredIntermediateStateAt":0,"intermediateStateTimeout":10,"jsOnReceive":"","jsonPath":"$.MaxADC","lastJsonPathValue":"1023","lastPayload":"{\"ADC\":0, \"MaxADC\":1023, \"Offset\":0}","qos":1,"retained":false,"topic":"windpoint/adc","topicPub":"windpoint/m","updateLastPayloadOnPub":false,"id":"c5ab2bd4-1ced-4d34-bcf4-83e9db606361","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1552427672,"longId":14,"name":"vaneMaxADC","type":3},{"decimalPrecision":0,"displayPayloadValue":true,"maxValue":359.0,"minValue":0.0,"postfix":"°","prefix":"","progressColor":-1,"enableIntermediateState":false,"enablePub":true,"enteredIntermediateStateAt":0,"intermediateStateTimeout":10,"jsOnReceive":"","jsonPath":"$.Offset","lastJsonPathValue":"0","lastPayload":"{\"ADC\":0, \"MaxADC\":1023, \"Offset\":0}","qos":1,"retained":false,"topic":"windpoint/adc","topicPub":"windpoint/o","updateLastPayloadOnPub":false,"id":"3e365f91-778b-456e-b5cf-446c1aaefe1c","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1552427672,"longId":16,"name":"vaneOffset","type":3},{"iconOff":"ic_cloud_download","iconOn":"ic_cloud_download","offColor":-1,"onColor":-1,"payloadOff":"sensor","payloadOn":"sensor","enableIntermediateState":true,"enablePub":true,"enteredIntermediateStateAt":0,"intermediateStateTimeout":0,"jsOnReceive":"","jsonPath":"","lastPayload":"sensor","qos":1,"retained":false,"topic":"windpoint","topicPub":"","updateLastPayloadOnPub":true,"id":"df5bfda3-d3b7-4c92-8fe7-a425e20cfd95","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1552384187,"longId":9,"name":"Update","type":2},{"mainTextSize":"SMALL","postfix":"","prefix":"","textColor":-1,"enableIntermediateState":true,"enablePub":false,"enteredIntermediateStateAt":0,"intermediateStateTimeout":0,"jsOnReceive":"","jsonPath":"","lastPayload":"ADC:0","qos":1,"retained":false,"topic":"windpoint/debug","topicPub":"","updateLastPayloadOnPub":true,"id":"d8222a02-352e-48d0-b466-b78e9a382a19","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1552427535,"longId":11,"name":"Debug","type":1},{"iconOff":"ic_explore","iconOn":"ic_explore","offColor":-1,"onColor":-1,"payloadOff":"adc","payloadOn":"adc","enableIntermediateState":false,"enablePub":true,"enteredIntermediateStateAt":0,"intermediateStateTimeout":0,"jsOnReceive":"","jsonPath":"","lastPayload":"adc","qos":1,"retained":false,"topic":"windpoint","topicPub":"","updateLastPayloadOnPub":true,"id":"cf88116a-4130-45bc-b77f-f6a0895b5548","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1552427071,"longId":12,"name":"Read ADC","type":2},{"iconOff":"ic_flash_on","iconOn":"ic_flash_on","offColor":-1,"onColor":-1,"payloadOff":"reset","payloadOn":"reset","enableIntermediateState":true,"enablePub":true,"enteredIntermediateStateAt":0,"intermediateStateTimeout":0,"jsOnReceive":"","jsonPath":"","lastPayload":"reset","qos":1,"retained":false,"topic":"windpoint","topicPub":"","updateLastPayloadOnPub":true,"id":"aabe562f-5790-4d2a-ae3b-60526dfd4621","jsBlinkExpression":"","jsOnDisplay":"","jsOnTap":"","lastActivity":1540372437,"longId":10,"name":"Reset!","type":2}]
</details>

MQTT Dash screenshot:<br>
![alt tag](https://raw.githubusercontent.com/zpukr/esp8266-WindStation/master/Screenshot_20190216.png)
