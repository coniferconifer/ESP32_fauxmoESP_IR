// IR JSON data generetor for ESP32_fauximoESP_IR_recorder.ino
// by coniferconifer http://github.com/coniferconifer
//
// This program waits for IR signal input from VS1383/CHQ1383 IR receiver connected to
// GIPIO35 and prints out JSON format data compatible to IRKit http://getirkit.com/en/
// every 10sec , should be used for alexa IR remote controller ESP32_fauximoESP_IR_sender.ino
//
/* Author: coniferconifer
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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"

#define RMT_TX_CHANNEL RMT_CHANNEL_4
#define RMT_TX_GPIO_NUM GPIO_NUM_33

#define RMT_RX_CHANNEL RMT_CHANNEL_0
#define RMT_RX_GPIO_NUM GPIO_NUM_35

#define RMT_CLK_DIV 100
#define RMT_TICK_10_US (80000000/RMT_CLK_DIV/100000)
#define rmt_item32_TIMEOUT_US 10000

#define MAX_SIGNAL_LEN 1024


void setup() {
  Serial.begin(115200);
  init_tx();
  init_rx();
}

bool ir_use = false;
size_t received = 0;
void loop() {
  Serial.println("IR receiver standby");
  process();

}


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



void rmt_rx_task(void *) {
  RingbufHandle_t rb = NULL;
  rmt_get_ringbuf_handle(RMT_RX_CHANNEL, &rb);
  rmt_rx_start(RMT_RX_CHANNEL, 1);

  size_t rx_size = 0;
  Serial.println("standing by to receive IR remote control signal...");
  rmt_item32_t *item = (rmt_item32_t*)xRingbufferReceive(rb, &rx_size, 3000);
  rmt_rx_stop(RMT_RX_CHANNEL);
  if (!item) {
    Serial.println("no data received");
    ir_use = false;
    vTaskDelete(NULL);
    return;
  }

#define NECANALYZER
#ifdef NECANALYZER
  if (item) {
    uint16_t rmt_addr;
    uint16_t rmt_cmd;
    int offset = 0;
    while (1) {
      Serial.println("NEC parser");
      //parse data value from ringbuffer.
      int res = nec_parse_items(item + offset, rx_size / 4 - offset, &rmt_addr, &rmt_cmd);
      //     Serial.print("res="); Serial.println(res);
      if (res > 0) {
        offset += res + 1;
        Serial.printf("RMT RCV --addr: 0x%04x cmd: 0x%04x\n", rmt_addr, rmt_cmd);
        ESP_LOGI(NEC_TAG, "RMT RCV --- addr: 0x%04x cmd: 0x%04x", rmt_addr, rmt_cmd);
      } else {
        break;
      }
    }

  }
#endif

  Serial.print("rmt_rx_task() received items: ");
  Serial.println(rx_size);

  memcpy(signals, item, sizeof(rmt_item32_t) * rx_size);
  for (int i = 0; i < rx_size; ++i) {
    signals[i].level0 = ~signals[i].level0;
    signals[i].level1 = ~signals[i].level1;
  }
  received = rx_size;
  vRingbufferReturnItem(rb, (void*)item);

  Serial.println("rmt_rx_task() done");

  rmt_rx_stop(RMT_RX_CHANNEL);
  ir_use = false;
  vTaskDelete(NULL);
}

#include <ArduinoJson.h>
StaticJsonBuffer<2000> jsonBuffer;
char json[2000];

void process() {
  char buf[MAX_SIGNAL_LEN * 4];
  int i; uint16_t duration, durationinusec; uint16_t highb, lowb;
  int datasize = sizeof(rmt_item32_t) * received;
  Serial.printf("rmt_item32_ size=%6d \n", sizeof(rmt_item32_t));
  Serial.printf("received=%6d \n", received);
  uint8_t* signaldata;
  signaldata = (uint8_t *)signals;
  Serial.printf("datasize=%6d \n", datasize);
  if ( datasize != 0 ) {
    Serial.printf("\n\rString airconon = \"{\\\"message\\\":{\\\"format\\\":\\\"raw\\\",\\\"freq\\\":38,\\\"data\\\":\\\n\r");
    Serial.printf("[");
    Serial.println("cut and paste JSON data into ESP32_fauxmoESP_Basic_IR.ino");
  }
  for (i = 0; i < datasize / 8; i++) {
    lowb = (*signaldata++) & 0xff ;
    highb = (*signaldata++) & 0xff;
    //      Serial.printf("%02x", highb&0x7f);Serial.printf("%02x ", lowb);
    duration = highb * 256 + lowb;
    durationinusec = 2 * ((duration & 0x7fff) * 10 / RMT_TICK_10_US);// irkit compatible
    if (i == 0) {
      Serial.printf("%d", durationinusec);
    } else {
      if (durationinusec < 40000) Serial.printf(",%d", durationinusec);
    }
    if ((i % 16) == 15 ) Serial.println("\\");
  }
  if ( datasize != 0 ) {
    Serial.print("]},\\\"hostname\\\":\\\"IRKitD2A4\\\",\\\"deviceid\\\":\\\"XXXXXXXX\\\"}\";\r\n\r\n");
    delay(5000);
  }

  ir_use = true;
  xTaskCreate(rmt_rx_task, "rmt_rx_task", 2048, NULL, 10, NULL);
  delay(5000);
}
#define NEC_HEADER_HIGH_US    9000                         /*!< NEC protocol header: positive 9ms */
#define NEC_HEADER_LOW_US     4500                         /*!< NEC protocol header: negative 4.5ms*/
#define NEC_BIT_ONE_HIGH_US    560                         /*!< NEC protocol data bit 1: positive 0.56ms */
#define NEC_BIT_ONE_LOW_US    (2250-NEC_BIT_ONE_HIGH_US)   /*!< NEC protocol data bit 1: negative 1.69ms */
#define NEC_BIT_ZERO_HIGH_US   560                         /*!< NEC protocol data bit 0: positive 0.56ms */
#define NEC_BIT_ZERO_LOW_US   (1120-NEC_BIT_ZERO_HIGH_US)  /*!< NEC protocol data bit 0: negative 0.56ms */
#define NEC_BIT_END            560                         /*!< NEC protocol end: positive 0.56ms */
#define NEC_BIT_MARGIN         60                          /*!< NEC parse margin time */

#define NEC_ITEM_DURATION(d)  ((d & 0x7fff)*10/RMT_TICK_10_US)  /*!< Parse duration time from memory register value */
#define NEC_DATA_ITEM_NUM   34  /*!< NEC code item number: header + 32bit data + end */
#define RMT_TX_DATA_NUM  100    /*!< NEC tx test data number */
#define rmt_item32_tIMEOUT_US 9500 /*!< RMT receiver timeout value(us) */
#define RMT_RX_ACTIVE_LEVEL 0 /*!< If we connect with a IR receiver, the data is active low */

/*
   @brief Check whether this value represents an NEC data bit 0
*/
static bool nec_bit_zero_if(rmt_item32_t* item)
{
  //  Serial.println("nec_bit_zero_if()");
  if ((item->level0 == RMT_RX_ACTIVE_LEVEL && item->level1 != RMT_RX_ACTIVE_LEVEL)
      && nec_check_in_range(item->duration0, NEC_BIT_ZERO_HIGH_US, NEC_BIT_MARGIN)
      && nec_check_in_range(item->duration1, NEC_BIT_ZERO_LOW_US, NEC_BIT_MARGIN)) {
    return true;
  }
  //  Serial.println("false");
  return false;
}
/*
   @brief Check whether this value represents an NEC data bit 1
*/
static bool nec_bit_one_if(rmt_item32_t* item)
{
  //  Serial.println("nec_bit_one_if()");
  if ((item->level0 == RMT_RX_ACTIVE_LEVEL && item->level1 != RMT_RX_ACTIVE_LEVEL)
      && nec_check_in_range(item->duration0, NEC_BIT_ONE_HIGH_US, NEC_BIT_MARGIN)
      && nec_check_in_range(item->duration1, NEC_BIT_ONE_LOW_US, NEC_BIT_MARGIN)) {
    return true;
  }
  //  Serial.println("false");
  return false;
}

/*
   @brief Check whether duration is around target_us
*/
inline bool nec_check_in_range(int duration_ticks, int target_us, int margin_us)
{
  //  Serial.println("nec_check_in_range():"); Serial.print( NEC_ITEM_DURATION(duration_ticks)); Serial.print(" ");
  //  Serial.print(target_us); Serial.print(" "); Serial.println(margin_us);
  if (( NEC_ITEM_DURATION(duration_ticks) < (target_us + margin_us))
      && ( NEC_ITEM_DURATION(duration_ticks) > (target_us - margin_us))) {
    return true;
  } else {
    //    Serial.println("false");
    return false;
  }
}
/*
   @brief Check whether this value represents an NEC header
*/
static bool nec_header_if(rmt_item32_t* item)
{
  //  Serial.println("nec_header_if()");
  if ((item->level0 == RMT_RX_ACTIVE_LEVEL && item->level1 != RMT_RX_ACTIVE_LEVEL)
      && nec_check_in_range(item->duration0, NEC_HEADER_HIGH_US, NEC_BIT_MARGIN)
      && nec_check_in_range(item->duration1, NEC_HEADER_LOW_US, NEC_BIT_MARGIN)) {
    //    Serial.println("true");
    return true;
  }
  //  Serial.println("false");
  return false;
}
/*
   @brief Parse NEC 32 bit waveform to address and command.
*/
static int nec_parse_items(rmt_item32_t* item, int item_num, uint16_t* addr, uint16_t* data)
{

  int w_len = item_num;
  if (w_len < NEC_DATA_ITEM_NUM) {
    return -1;
  }
  int i = 0, j = 0;
  if (!nec_header_if(item++)) {
    return -1;
  }
  //  Serial.println("NEC parser bit");
  uint16_t addr_t = 0;
  for (j = 0; j < 16; j++) {
    if (nec_bit_one_if(item)) {
      addr_t |= (1 << j);
    } else if (nec_bit_zero_if(item)) {
      addr_t |= (0 << j);
    } else {
      return -1;
    }
    item++;
    i++;
  }
  uint16_t data_t = 0;
  for (j = 0; j < 16; j++) {
    if (nec_bit_one_if(item)) {
      data_t |= (1 << j);
    } else if (nec_bit_zero_if(item)) {
      data_t |= (0 << j);
    } else {
      return -1;
    }
    item++;
    i++;
  }
  *addr = addr_t;
  *data = data_t;
  return i;
}

