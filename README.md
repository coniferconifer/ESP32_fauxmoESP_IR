## IR remote commander for Alexa (Amazon echo) by ESP32
 Thanks to fauxmoESP , voice command can send IR signal to control nearby appliances.
 Change json data in the source code for the device to be controlled by IR remote controller.
 
### How to get IR command Json data 
- step 1 setup serial terminal software (teraterm etc) , set speed 115200bps.
- step 2 run ESP32_fauxmoESP_IR_recorder.ino
- step 3 send IR signal by pressing a button on your remote controller.
- step 4 copy the json data recorded by ESP32_fauxmoESP_IR_recorder.ino and past it into ESP32_fauxmoESP_sender.ino
- step 5 prepare credentials.h for your WiFi AP and run ESP32_fauxmoESP_IR_sender.ino 
         this program appears as "switch one" in Alexa application
- step 6 install Alexa application from GooglePlay and add "switch one" as "aircon" from smarthome menu.
- step 7 wait for few seconds to connect to WiFi access point and recognized from Amazon Echo
- step 8 speak to Amazon Echo "turn on aircon", "turn off aircon"

```
ESP32_fauxmoESP_IR_sender
20180504
[WIFI] Connecting to aterm-3933 ........
[WIFI] STATION Mode, SSID: aterm-3933, IP address: 192.168.13.11
[MAIN] Device #0 (switch one) state: ON
{"message":{"format":"raw","freq":38,"data":[7620,3744,980,894,984,2794,984,892,984,2794,982,894,984,2792,984,894,984,2792,984,894,984,2794,982,894,984,2794,982,2792,982,894,986,2792,982,894,986,2794,980,2792,982,2794,982,2794,982,894,984,894,984,2794,982,2792,982,894,986,894,984,894,984,894,986,2790,984,894,984,894,984,896,984,2794,982,2792,984,2790,984,894,984,894,984,894,984,894,986,894,984,2792,984,894,984,894,984,894,984,2794,984,892,984,894,986,892,984,896,984,2794,982,894,984,894,984,894,986,2786,986,894,984,894,986,894,984,894,984,894,986,892,986,894,984,894,986,892,984,894,986,892,986,894,984,894,984,2794,982,894,986,892,986,892,986,894,984,894,986,892,986,894,984,894,984,894,984,894,986,892,986,2792,984,894,982,896,984,894,984,894,984,896,984,894,984,894,984,894,984,896,986,892,984,2792,984,894,984,2792,984,2790,984,2792,984,2794,982,2790,984,894,986,892,986,894,984,894,984,894,984,894,984,896,984,0]},"hostname":"IRKitD2A4","deviceid":"XXXXXXXX"}
Json parsed
rmt_tx_task created
send.............  done
[MAIN] Free heap: 134524 bytes
[MAIN] Device #0 (switch one) state: OFF
```

### what's new
- Json data for NEC sealing light and SHARP air conditioner are included in the source.

![ESP32_fauxmoESP](https://github.com/coniferconifer/ESP32_fauxmoESP_IR/blob/master/fauxmoESP.jpg)

## License: Apache License v2

## hardware: 
- CHQ1838 IR receiver output is connected to GPIO 35
- IR LED is connected to GPIO 33 , driven by NPN transiter 2N5551
  
## References

### fauxmoESP: Amazon Alexa support for ESP8266 and ESP32 devices.  
- [https://bitbucket.org/xoseperez/fauxmoesp](https://bitbucket.org/xoseperez/fauxmoesp)

### How to use IR sender / receiver on ESP32
- [https://qiita.com/td2sk/items/4c0ef83bcc7e74e5e8d5](https://qiita.com/td2sk/items/4c0ef83bcc7e74e5e8d5)
### sender/receiver code for ESP32
- [https://github.com/espressif/esp-idf/blob/master/examples/peripherals/rmt_nec_tx_rx/main/infrared_nec_main.c](https://github.com/espressif/esp-idf/blob/master/examples/peripherals/rmt_nec_tx_rx/main/infrared_nec_main.c)  
### Json format for IR controller is compatible with IRKit
- [http://getirkit.com/en/](http://getirkit.com/en/)
  
### sigrok logic analyzer
- [https://sigrok.org/wiki/PulseView](https://sigrok.org/wiki/PulseView)
- [http://www.geeetech.com/wiki/index.php/CY7C68013](http://www.geeetech.com/wiki/index.php/CY7C68013)

![NEC light remocon received](https://github.com/coniferconifer/ESP32_fauxmoESP_IR/blob/master/NEC_light.png)

![CY7C68013A-56 EZ-USB FX2LP USB board for sigrok](https://github.com/coniferconifer/ESP32_fauxmoESP_IR/blob/master/sigrok.JPG)
