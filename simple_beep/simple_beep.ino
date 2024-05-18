/*MUSE beep
 *   
 *  This is a demo code to make a beep when the IO32 (touch9) button is touched using your finger, so no need to use a push button. IO32 is located on the upper left part of the Proto Board.
 *  
 *  intended for the esp32 wrover board with dac an amp ESP MUSE PROTO
 *  https://raspiaudio.com/produit/muse-proto
 *  
    the beep.wav file needs to be uploaded to SPIFFS the file is located in the /data directory of this squetch
    then use the arduino ESP32 squetch data uplaad section

    The wave file must be encoded on 8 bits

    For the tutorial https://forum.raspiaudio.com/t/muse-proto-board/231
*/

#include "Arduino.h"
#include "SPIFFS.h"
#include "driver/i2s.h"
#include "driver/gpio.h"
#include <Adafruit_NeoPixel.h>

#define I2SR (i2s_port_t)0
#define PW GPIO_NUM_21  // Amp power ON
#define GAIN GPIO_NUM_23
#define BLOCK_SIZE 128
#define TOUCH_PIN T9  // GPIO32

const uint16_t PixelCount = 1;
const uint8_t PixelPin = 22;
Adafruit_NeoPixel pixels(PixelCount, PixelPin, NEO_GRB + NEO_KHZ800);

File wavFile;
int threshold = 40;
bool touchdetected = false;

void gotTouch() {
  touchdetected = true;
}

const i2s_config_t i2s_configR = {
  .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_TX),
  .sample_rate = 44100,
  .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
  .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
  .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
  .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
  .dma_buf_count = 4,
  .dma_buf_len = BLOCK_SIZE
};

i2s_pin_config_t pin_configR = {
  .bck_io_num = 5,
  .ws_io_num = 25,
  .data_out_num = 26,
  .data_in_num = 35
};

void playWAV() {
  if (!wavFile) {
    Serial.println("No WAV file is open");
    return;
  }

  uint8_t buffer[BLOCK_SIZE];
  size_t bytesRead, bytesWritten;

  // Skip the WAV header
  wavFile.seek(44);

  while (wavFile.available()) {
    bytesRead = wavFile.read(buffer, BLOCK_SIZE);
    i2s_write(I2SR, buffer, bytesRead, &bytesWritten, portMAX_DELAY);
  }
}


void setup() {
  Serial.begin(115200);
  esp_err_t err;

  // Initialize SPIFFS
  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS initialization failed");
    pixels.setPixelColor(0, pixels.Color(150, 0, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.
    return;
  }
  Serial.println("SPIFFS initialized");


  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'


  // Find the first WAV file in the root directory
  File root = SPIFFS.open("/");
  wavFile = root.openNextFile();
  while (wavFile) {
    if (!wavFile.isDirectory() && String(wavFile.name()).endsWith(".wav")) {
      Serial.print("Found WAV file: ");
      Serial.println(wavFile.name());
      break;
    }
    wavFile = root.openNextFile();
  }

  if (!wavFile) {
    Serial.println("No WAV files found in spiff");
    pixels.setPixelColor(0, pixels.Color(150, 0, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.

    return;
  }

  // Amp power enable
  gpio_reset_pin(PW);
  gpio_set_direction(PW, GPIO_MODE_OUTPUT);
  gpio_set_level(PW, 0);

  gpio_reset_pin(GAIN);
  gpio_set_direction(GAIN, GPIO_MODE_OUTPUT);
  gpio_set_pull_mode(GAIN, GPIO_PULLDOWN_ONLY);

  // I2S port0 init: TX, RX, mono, 16bits, 44100hz
  i2s_driver_install(I2SR, &i2s_configR, 0, NULL);
  i2s_set_pin(I2SR, &pin_configR);
  i2s_start(I2SR);


  pixels.setPixelColor(0, pixels.Color(0, 150, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.


  //Enable amp
  gpio_set_level(PW, 1);
  playWAV();
  gpio_set_level(PW, 0);

  // Initialize touch pad
  touchAttachInterrupt(TOUCH_PIN, gotTouch, threshold);

}

void loop() {
  if (touchdetected) {
    // Initialize touch pad
    touchDetachInterrupt(TOUCH_PIN);

    pixels.setPixelColor(0, pixels.Color(150, 150, 150));
    pixels.show();   // Send the updated pixel colors to the hardware.


    touchdetected = false;
    Serial.println("Touch detected, playing WAV file...");

    //Enable amp
    gpio_set_level(PW, 1);
    playWAV();
    gpio_set_level(PW, 0);

    pixels.setPixelColor(0, pixels.Color(0, 150, 0));
    pixels.show();   // Send the updated pixel colors to the hardware.


    touchAttachInterrupt(TOUCH_PIN, gotTouch, threshold);
  }
  delay(100); // Debounce delay
}
