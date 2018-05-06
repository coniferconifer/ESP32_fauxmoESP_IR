# IR remote commander for Alexa (Amazon echo) by ESP32

## change json data in the source code to device to be controlled by IR remote controller
- step 1 setup serial terminal software (teraterm etc) , set speed 115200bps.
- step 2 run ESP32_fauxmoESP_IR_recorder.ino
- step 3 send IR signal by pressing a button on your remote controller.
- step 2 copy and paste the json data recorded by ESP32_fauxmoESP_IR_recorder.ino

### what's new
- Json data for NEC sealing light and SHARP air conditioner are included in the source.

![ESP32_fauxmoESP](https://github.com/coniferconifer/ESP32_faxumoESP/blob/master/fauxmoESP.jpg)

## License: Apache License v2

## hardware: 
- CHQ1838 IR receiver output is connected to GPIO 35
- IR LED is connected to GPIO 33 , driven by NPN transiter 2N5551
  
## References

### fauxmoESP: Amazon Alexa support for ESP8266 and ESP32 devices.  
  [https://bitbucket.org/xoseperez/fauxmoesp](https://bitbucket.org/xoseperez/fauxmoesp)
