/**
   Alexa IR remote controller based on fauximoESP using ESP32 IR remote transimitter
   for SHARP air conditioner or NEC sealing light.
   Install Alexa application in your Android machine and assign "switch one" as aircon.
   Speak to Amazon echo "Alexa, turn on aircon" or "Alexa, turn off aircon"

   GPIO_NUM_33 is used to send IR remote control pulse using 2N5551 NPN transister
   GPIO_NUM_33 is HIGH , then IR LED is OFF.
   GPIO_NUM_33 is LOW , then IR LED is ON.
   GPIO_NUM_4 is used to indicate ON/OFF state

   Author: coniferconifer
   License: Apache License v2
   May 2,2018

  Reference
  [1] fauximoESP "Amazon Alexa support for ESP8266 and ESP32 devices."
  https://bitbucket.org/xoseperez/fauxmoesp

  [2] code for IR remote controller on ESP32
  https://qiita.com/td2sk/items/4c0ef83bcc7e74e5e8d5
  https://github.com/espressif/esp-idf/blob/master/examples/peripherals/rmt_nec_tx_rx/main/infrared_nec_main.c

  [3] Json format for IR controller is compatible with IRkit
  http://getirkit.com/en/
*/
#include <Arduino.h>
#define ESP32

#include <WiFi.h>
#include "fauxmoESP.h"
#include "credentials.h"

#define SERIAL_BAUDRATE                 115200
#define LED                             GPIO_NUM_4


//NEC light on/off
String lightonjson = "{ \"message\": {\"format\": \"raw\", \"freq\": 38, \"data\": [ \
      18162, 8816, 1252, 1020, 1246, 3232, 1242, 3240, 1240, 970, 1292, 974, 1294, 920, 1244, 1020, \
      1246, 3232, 1246, 966, 1244, 1020, 1242, 972, 1300, 966, 1244, 966, 1246, 1016, 1246, 3236, 1240, \
      970, 1244, 1020, 1244, 3236, 1244, 966, 1296, 3180, 1250, 3234, 1246, 1016, 1244, 970, 1244, 1020\
      , 1244, 3232, 1254, 956, 1246, 3236, 1242, 1022, 1246, 966, 1244, 3234, 1246, 3234, 1244, 3234, \
      1296, 0]}, \"hostname\": \"IRKitD2A4\", \"deviceid\": \"yyyyyyyyyyyyy\"}";

//  SHARP Aircon Hot on
/*String airconjson = "{\"message\":{\"format\":\"raw\",\"freq\":38,\"data\":[\
  7622,3736,986,894,984,2794,980,896,984,2792,982,894,986,2794,976,896,984,2792,984,892,986,2796,\
  980,894,986,2794,980,2794,982,894,982,2796,980,894,986,2790,984,2792,984,2790,984,2794,982,894,\
  984,896,982,2794,982,2794,980,896,984,896,984,892,984,896,982,2800,980,894,984,894,984,894,984,\
  2794,980,896,984,2792,984,892,986,894,986,892,986,892,986,894,984,2790,984,894,986,892,990,892,\
  986,2792,982,896,982,896,982,892,1042,2740,984,892,990,890,986,894,986,894,982,2796,982,892,1040,\
  840,986,892,984,894,982,896,992,892,934,944,984,894,936,942,984,894,936,2840,982,894,940,944,934,\
  2844,930,946,934,944,982,894,940,942,980,896,940,942,940,944,934,944,934,946,980,896,936,942,940,\
  2834,940,940,942,940,940,942,936,942,936,942,936,942,940,940,942,940,940,936,940,946,934,2836,936,\
  942,940,2836,982,2794,936,2840,936,2834,940,2840,934,942,992,892,936,942,934,942,940,940,942,936,\
  942,940,940,0]},\"hostname\":\"IRKitD2A4\",\"deviceid\":\"xxxxxxxxxxxx\"}";
*/
String airconjson = "{\"message\":{\"format\":\"raw\",\"freq\":38,\"data\":\
[7620,3744,980,894,984,2794,984,892,984,2794,982,894,984,2792,984,894\
,984,2792,984,894,984,2794,982,894,984,2794,982,2792,982,894,986,2792\
,982,894,986,2794,980,2792,982,2794,982,2794,982,894,984,894,984,2794\
,982,2792,982,894,986,894,984,894,984,894,986,2790,984,894,984,894\
,984,896,984,2794,982,2792,984,2790,984,894,984,894,984,894,984,894\
,986,894,984,2792,984,894,984,894,984,894,984,2794,984,892,984,894\
,986,892,984,896,984,2794,982,894,984,894,984,894,986,2786,986,894\
,984,894,986,894,984,894,984,894,986,892,986,894,984,894,986,892\
,984,894,986,892,986,894,984,894,984,2794,982,894,986,892,986,892\
,986,894,984,894,986,892,986,894,984,894,984,894,984,894,986,892\
,986,2792,984,894,982,896,984,894,984,894,984,896,984,894,984,894\
,984,894,984,896,986,892,984,2792,984,894,984,2792,984,2790,984,2792\
,984,2794,982,2790,984,894,986,892,986,894,984,894,984,894,984,894\
,984,896,984,0]},\"hostname\":\"IRKitD2A4\",\"deviceid\":\"XXXXXXXX\"}";

/*String airconjson = "{\"message\":{\"format\":\"raw\",\"freq\":38,\"data\":[\
  7624,3736,984,892,986,2792,984,892,990,2792,982,892,990,2790,984,892,986,2794,982,892,986,2790,\
  984,894,986,2792,982,2792,984,894,984,2790,984,894,990,2784,986,2796,980,2844,932,2790,984,894,\
  984,894,986,2792,984,2794,980,892,986,894,986,892,986,892,986,2844,932,892,984,894,986,892,986,\
  2792,984,2790,986,2786,990,890,986,894,986,892,986,892,990,890,990,2786,986,894,986,892,986,892,\
  990,2790,984,892,986,892,990,890,990,892,990,2784,986,894,986,892,990,890,1040,2734,990,892,986,\
  894,986,890,990,890,992,886,990,892,990,890,990,890,990,892,990,890,990,890,990,892,1040,840,990,\
  2790,984,892,990,892,1034,844,984,894,986,892,986,890,992,890,986,894,986,892,982,896,984,894,984,\
  2794,986,892,984,892,986,894,984,896,982,894,984,894,984,894,984,896,982,896,980,896,984,2796,984,\
  892,984,2794,934,2840,984,2794,982,2790,982,2794,980,902,934,942,936,942,940,940,942,940,940,940,\
  940,936,942,0]},\"hostname\":\"IRKitD2A4\",\"deviceid\":\"xxxxxxxxxx\"}"; */

String airconon = "{\"message\":{\"format\":\"raw\",\"freq\":38,\"data\":\
[7622,3740,986,892,986,2794,980,896,982,2794,982,894,984,2792,984,894\
,984,2786,990,892,986,2792,982,896,984,2794,980,2794,982,894,986,2792\
,980,896,984,2796,980,2792,984,2794,980,2796,982,892,986,892,984,2792\
,984,2794,980,896,984,890,990,894,984,896,982,2792,986,894,982,896\
,984,892,986,2790,986,2794,980,2794,980,896,984,894,986,894,982,894\
,986,896,982,2790,984,896,984,896,982,894,986,2794,982,894,982,894\
,986,894,984,894,986,2792,982,894,984,894,982,896,984,2794,984,892\
,986,892,986,894,984,894,984,896,982,894,984,896,984,896,982,892\
,986,894,986,896,982,894,984,894,986,2792,982,896,982,894,984,894\
,986,894,984,896,984,894,984,896,982,894,984,894,986,892,986,896\
,982,2796,976,896,984,894,984,896,982,894,986,894,984,896,982,894\
,986,894,984,894,982,896,984,2792,982,900,982,2794,982,2794,980,2794\
,982,2792,984,2790,984,896,982,896,984,896,982,894,984,894,986,894\
,984,894,982,0]},\"hostname\":\"IRKitD2A4\",\"deviceid\":\"XXXXXXXX\"}";

String airconoffjson = "{\"message\":{\"format\":\"raw\",\"freq\":38,\"data\":[\
7636,3722,1062,820,1054,2722,1060,822,1060,2714,1064,816,1054,2720,1056,824,1056,2722,1054,816,1004,\
2772,1064,814,1062,2716,1064,2706,1066,814,1056,2722,1060,820,1060,2714,1064,2710,1066,2706,1064,2716,\
1060,814,1066,814,1060,2714,1064,2712,1066,810,1074,812,1070,806,1062,816,1066,2706,1074,806,1070,806,\
1070,814,1066,2710,1066,812,1066,2712,1062,814,1066,810,1064,820,1064,810,1064,820,1066,2704,1070,814,\
1066,810,1070,812,1066,812,1062,2714,1072,806,1072,810,1070,2704,1070,812,1070,806,1066,814,1066,812,\
1122,2650,1074,806,1070,810,1074,806,1074,806,1064,812,1122,760,1072,810,1066,812,1066,812,1072,810,\
1070,2704,1070,810,1072,804,1072,2706,1070,810,1070,810,1072,806,1074,804,1074,806,1064,812,1122,762,\
1072,804,1070,814,1074,802,1070,812,1070,2706,1066,814,1072,802,1080,802,1072,812,1066,810,1076,800,\
1070,812,1070,810,1072,806,1074,804,1076,2700,1076,804,1070,2704,1074,2700,1074,2702,1074,2700,1074,\
2702,1076,806,1070,806,1074,806,1070,2704,1076,2700,1076,802,\
1072,810,1124,0]},\"hostname\":\"IRKitD2A4\",\"deviceid\":\"xxxxxxxxxxxxx\"}";

void wifiSetup() {
  WiFi.mode(WIFI_STA);
  Serial.printf("[WIFI] Connecting to %s ", WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS); // defined in credentials.h
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  Serial.printf("[WIFI] STATION Mode, SSID: %s, IP address: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());

}

fauxmoESP fauxmo;
void setup() {

  Serial.begin(SERIAL_BAUDRATE);
  Serial.print("ESP32_fauxmoESP_IR_sender: ");
  Serial.println("20180506");

  wifiSetup();

  pinMode(GPIO_NUM_33, OUTPUT); // for IR LED driven by 2N5551 NPN transister
  digitalWrite(GPIO_NUM_33, HIGH); delay(1000);
  // IR remocon initialize
  initir();
  // LED
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(1000);
  digitalWrite(LED, LOW);
  // process(airconjson); //test
  fauxmo.addDevice("switch one");
  fauxmo.enable(true);
  fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state) {
    Serial.printf("[MAIN] Device #%d (%s) state: %s\n", device_id, device_name, state ? "ON" : "OFF");
    digitalWrite(LED, !state);
    if (state == 1) process(airconjson); //turn on
    else process(airconoffjson); //turn off

  });

  // Callback to retrieve current state (for GetBinaryState queries)
  fauxmo.onGetState([](unsigned char device_id, const char * device_name) {
    return !digitalRead(LED);
  });

}

void loop() {
  fauxmo.handle();
  static unsigned long last = millis();
  if (millis() - last > 5000) {
    last = millis();
    Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
  }

}
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"
#include "http_parser.h"

#define RMT_TX_CHANNEL RMT_CHANNEL_4
#define RMT_TX_GPIO_NUM GPIO_NUM_33

#define RMT_RX_CHANNEL RMT_CHANNEL_0
#define RMT_RX_GPIO_NUM GPIO_NUM_35

#define RMT_CLK_DIV 100
#define RMT_TICK_10_US (80000000/RMT_CLK_DIV/100000)
#define rmt_item32_TIMEOUT_US 10000

#define MAX_SIGNAL_LEN 1024

void initir() {
  init_tx();
  //  init_rx();
}

/* ir */
bool ir_use = false;
size_t received = 0;
rmt_item32_t signals[MAX_SIGNAL_LEN];

void init_tx() {
  rmt_config_t rmt_tx;
  rmt_tx.rmt_mode = RMT_MODE_TX;
  rmt_tx.channel = RMT_TX_CHANNEL;
  rmt_tx.gpio_num = RMT_TX_GPIO_NUM;
  rmt_tx.mem_block_num = 4;
  rmt_tx.clk_div = RMT_CLK_DIV;
  rmt_tx.tx_config.loop_en = false;
  rmt_tx.tx_config.carrier_duty_percent = 50;
  rmt_tx.tx_config.carrier_freq_hz = 38000;
  rmt_tx.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  rmt_tx.tx_config.carrier_en = 1;
  rmt_tx.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
  rmt_tx.tx_config.idle_output_en = true;
  rmt_config(&rmt_tx);
  rmt_driver_install(rmt_tx.channel, 0, 0);
}
/*
  void init_rx() {
  rmt_config_t rmt_rx;
  rmt_rx.rmt_mode = RMT_MODE_RX;
  rmt_rx.channel = RMT_RX_CHANNEL;
  rmt_rx.clk_div = RMT_CLK_DIV;
  rmt_rx.gpio_num = RMT_RX_GPIO_NUM;
  rmt_rx.mem_block_num = 4;
  rmt_rx.rx_config.filter_en = true;
  rmt_rx.rx_config.filter_ticks_thresh = 100;
  rmt_rx.rx_config.idle_threshold = rmt_item32_TIMEOUT_US / 10 * (RMT_TICK_10_US);
  rmt_config(&rmt_rx);
  rmt_driver_install(rmt_rx.channel, 1000, 0);
  }
*/
void rmt_tx_task(void *) {
  int i;
  Serial.print("send...");
  for (i = 0; i < 10; i++) {
    Serial.print("."); delay(10); // minimum frame space should be more than 8msec
    rmt_write_items(RMT_TX_CHANNEL, signals, received, true);
    rmt_wait_tx_done(RMT_TX_CHANNEL, portMAX_DELAY);
  }
  Serial.println("  done");
  ir_use = false;
  vTaskDelete(NULL);
}

#include <ArduinoJson.h>
StaticJsonBuffer<1000> jsonBuffer;

void process(String json) {

//  Serial.println(json);
  StaticJsonBuffer<5000> jsonBuffer;
  JsonObject& root = jsonBuffer.parseObject(json);
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }
  root.printTo(Serial);
  int i = 0;
  uint16_t* pointer;
  pointer = (uint16_t*) signals;
  Serial.println("\nJson parsed");
  while (1) {
    int value = root["message"]["data"][i];
    value = value / 2 * RMT_TICK_10_US / 10;
    //    Serial.print(value); Serial.print(" "); Serial.print(i); Serial.print(":");
    *pointer = value;
    i++;
//    Serial.printf("*%d \n", *pointer);
    if (i % 2 == 1) *pointer = *pointer | 0x8000;
    pointer++;
    if (value == 0) break;
  }
  received = i / 2;
//  Serial.printf("\nreceived = %8d\n", received);
 
    ir_use = true;
    Serial.println("rmt_tx_task created");
    xTaskCreate(rmt_tx_task, "rmt_tx_task", 2048, NULL, 10, NULL);

  
}


