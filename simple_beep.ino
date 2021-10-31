/* MUSE beep
 *   
 *  THis is a demo code to make a beep when the IO0 button is pressed
 *  
 *  intended for the esp32 wrover board with dac an amp ESP MUSE PROTO
 *  https://raspiaudio.com/produit/muse-proto
 *  
    the beep.wav file needs to be uploaded to SPIFFS the file is locataed in the /data directory of this squetch
    then use the arduino ESP32 squetch data uplaad section

    tfor the tutorial https://forum.raspiaudio.com/t/muse-proto-board/231


   This code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
//////////////////////////////////////////////////////////////////////////
//
// BEEP
//
//////////////////////////////////////////////////////////////////////////
#include "Audio.h"
#include "Arduino.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "esp_log.h"
//#include <sys/socket.h>
//#include <dirent.h>
#include <driver/i2s.h>
#include <NeoPixelBus.h>
#include "SPIFFS.h"

#define I2SR (i2s_port_t)0
#define VM GPIO_NUM_0        // button
#define ONled 0
#define PW GPIO_NUM_21        // Amp power ON
#define BLOCK_SIZE 128


RgbColor REDL(64, 0, 0);
RgbColor GREENL(0, 64, 0);
RgbColor BLUEL(0, 0, 64);
RgbColor WHITEL(64, 64, 64);
RgbColor BLACKL(0, 0, 0);


bool testOK;
static bool mp3ON;
const uint16_t PixelCount = 1;
const uint8_t PixelPin = 22;

// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbFeature, Neo800KbpsMethod> strip(PixelCount, PixelPin);



const i2s_config_t i2s_configR = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX ), // Receive, transfer
  .sample_rate = 44100,                         // 16KHz
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, //
  .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT, // although the SEL config should be left, it seems to transmit on right
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,     // Interrupt level 1
  .dma_buf_count = 4,                           // number of buffers
  .dma_buf_len = BLOCK_SIZE                           // samples per buffer
};



i2s_pin_config_t pin_configR =
{
  .bck_io_num = 5 ,    // BCKL
  .ws_io_num = 25 ,    // LRCL
  .data_out_num = 26,  // DOUT
  .data_in_num = 35    // DIN
};



//////////////////////////////////////////////////////////////////////////
//led refresh for Muse PROTO (1 leds ws2812)
/////////////////////////////////////////////////////////////////////////
void ledRefresh(int n, uint32_t v, int s)
{
  strip.SetPixelColor(n, v & s);
  strip.Show();
}


////////////////////////////////////////////////////////////////////////////////////////
//
// Task playing audio signal (mono, 16 bits, 44100)
//
////////////////////////////////////////////////////////////////////////////////////////


static void playAudio(void* data)
{
  int16_t s0, s1;
  int8_t c[16000];
  int l;
  size_t t;
  uint16_t s16[64];

  int a = 0;
  i2s_set_clk(I2SR, 44100, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);

  File f = SPIFFS.open("/beep.wav", FILE_READ);
  if (f == NULL) printf("err opening file\n");
  // header read
  f.read((uint8_t*)c, 44);
  // data read
  l = (int)f.read((uint8_t*)c, 16000);
  if (l < 0) printf("Erreur SD\n");

  //  i2s_zero_dma_buffer(I2SR);
  for (int i = 0; i < l; i++)
  {
    // sample value 8bits -> 16
    s0 = (((int16_t)(c[i] & 0xFF)) - 128) << 8 ;
    // attenuation  a
    s0 = s0 >> a;
    // buffering -> s16
    s16[i % 64] = (uint16_t)s0;
    if (((i + 1) % 64) ==  0)
    {
      int n = 0;
      //sending
      while (n == 0) n = i2s_write_bytes(I2SR, (const char*)s16, 128, portMAX_DELAY);
    }
  }
  // muting after playing
  for (int i = 0; i < 64; i++)s16[i] = 0;
  int n = 0;
  while (n == 0) n = i2s_write_bytes(I2SR, (const char*)s16, 128, portMAX_DELAY);
  i2s_zero_dma_buffer(I2SR);

  f.close();
  printf ("Play End\n");
  vTaskDelete(NULL);
}

/////////////////////////////////////////////////////////////////////////////////////////
//
// APP init
/////////////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(115200);
  esp_err_t err;

  if (!SPIFFS.begin(true))Serial.println("Erreur SPIFFS");


  //Init button
  gpio_reset_pin(VM);
  gpio_set_direction(VM, GPIO_MODE_INPUT);
  gpio_set_pull_mode(VM, GPIO_PULLUP_ONLY);

  // Amp power enable
  gpio_reset_pin(PW);
  gpio_set_direction(PW, GPIO_MODE_OUTPUT);
  gpio_set_level(PW, 1);

  //I2S port0 init:   TX, RX, mono , 16bits, 44100hz
  i2s_driver_install(I2SR, &i2s_configR, 0, NULL);
  i2s_set_pin(I2SR, &pin_configR);
  i2s_stop(I2SR);

  //turn RGB led to white
  strip.Begin();
  strip.SetPixelColor(ONled, WHITEL);
  strip.Show();

}



void loop() {

  if (gpio_get_level(VM) == 0)
  {

    //turn RGB led to blue
    strip.SetPixelColor(ONled, BLUEL);
    strip.Show();

    //play beep
    i2s_start(I2SR);
    xTaskCreatePinnedToCore(playAudio, "playAudio", 20000, NULL, 10, NULL, 1);
    delay(2000);
    i2s_stop(I2SR);

    //turn RGB led to red
    strip.SetPixelColor(ONled, REDL);
    strip.Show();

  }

  delay(100);

}
