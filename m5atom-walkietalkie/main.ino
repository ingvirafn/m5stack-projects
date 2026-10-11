
#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <esp_err.h>
#include <driver/i2s_pdm.h>
#include <driver/i2s_std.h>
#include <Adafruit_NeoPixel.h>
#include "AudioPacket.h"

// ====================================
// M5Stack ATOM Echo Walkie-Talkie
// Simple playback + automatic mute
// ====================================

#define BUTTON_PIN 39
#define LED_PIN 27

#define MIC_DATA GPIO_NUM_23
#define MIC_CLK GPIO_NUM_33

#define SPK_DATA GPIO_NUM_22
#define SPK_BCLK GPIO_NUM_19
#define SPK_LRCK GPIO_NUM_33

#define WIFI_CHANNEL 1

#define MIC_RATE 16000
#define AUDIO_RATE 8000

#define MIC_GAIN 2.0f

#define AUDIO_TIMEOUT_MS 80

// Small queue: max 3 waiting packets
#define QUEUE_SIZE 4

Adafruit_NeoPixel led(
  1, LED_PIN, NEO_GRB + NEO_KHZ800
);

uint8_t broadcastMac[6] = {
  0xFF, 0xFF, 0xFF,
  0xFF, 0xFF, 0xFF
};

i2s_chan_handle_t micHandle = nullptr;
i2s_chan_handle_t speakerHandle = nullptr;

bool micReady = false;
bool speakerReady = false;
bool speakerEnabled = false;

volatile bool transmitting = false;

AudioPacket audioQueue[QUEUE_SIZE];

volatile uint8_t queueHead = 0;
volatile uint8_t queueTail = 0;

volatile uint32_t lastPacketReceived = 0;

portMUX_TYPE queueMux =
  portMUX_INITIALIZER_UNLOCKED;

bool buttonPressed = false;
bool rawButtonPrevious = false;

uint32_t buttonChangeTime = 0;
uint32_t lastStatsTime = 0;

uint16_t sequenceNumber = 0;

uint32_t txPackets = 0;
volatile uint32_t rxPackets = 0;

float micDC = 0.0f;

// ====================================
// LED
// ====================================

void setLED(uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

void ledReady() {
  setLED(0, 45, 12);
}

void ledTransmit() {
  setLED(60, 0, 0);
}

void ledReceive() {
  setLED(0, 0, 60);
}

// ====================================
// Queue
// ====================================

void clearQueue() {
  portENTER_CRITICAL(&queueMux);
  queueHead = queueTail = 0;
  portEXIT_CRITICAL(&queueMux);
}

bool popPacket(AudioPacket &packet) {
  bool found = false;

  portENTER_CRITICAL(&queueMux);

  if (queueHead != queueTail) {
    packet = audioQueue[queueTail];
    queueTail =
      (queueTail + 1) % QUEUE_SIZE;
    found = true;
  }

  portEXIT_CRITICAL(&queueMux);

  return found;
}

// ====================================
// ESP-NOW RX
// ====================================

void onReceive(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int length
) {
  if (transmitting) return;
  if (length != sizeof(AudioPacket)) return;

  portENTER_CRITICAL(&queueMux);

  uint8_t next =
    (queueHead + 1) % QUEUE_SIZE;

  // If full, discard oldest audio.
  if (next == queueTail) {
    queueTail =
      (queueTail + 1) % QUEUE_SIZE;
  }

  memcpy(
    &audioQueue[queueHead],
    data,
    sizeof(AudioPacket)
  );

  queueHead = next;

  lastPacketReceived = millis();
  rxPackets++;

  portEXIT_CRITICAL(&queueMux);
}

// ====================================
// I2S stop
// ====================================

void stopMicrophone() {
  if (micHandle) {
    if (micReady) {
      i2s_channel_disable(micHandle);
    }

    i2s_del_channel(micHandle);
    micHandle = nullptr;
  }

  micReady = false;
}

void stopSpeaker() {
  if (speakerHandle) {
    if (speakerEnabled) {
      i2s_channel_disable(speakerHandle);
    }

    i2s_del_channel(speakerHandle);
    speakerHandle = nullptr;
  }

  speakerReady = false;
  speakerEnabled = false;
}

// ====================================
// Microphone init
// ====================================

bool startMicrophone() {
  stopSpeaker();
  stopMicrophone();

  i2s_chan_config_t channelConfig =
    I2S_CHANNEL_DEFAULT_CONFIG(
      I2S_NUM_0,
      I2S_ROLE_MASTER
    );

  esp_err_t err = i2s_new_channel(
    &channelConfig,
    nullptr,
    &micHandle
  );

  if (err != ESP_OK) return false;

  i2s_pdm_rx_config_t config = {};

  config.clk_cfg =
    I2S_PDM_RX_CLK_DEFAULT_CONFIG(MIC_RATE);

  config.slot_cfg =
    I2S_PDM_RX_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_16BIT,
      I2S_SLOT_MODE_MONO
    );

  config.gpio_cfg.clk = MIC_CLK;
  config.gpio_cfg.din = MIC_DATA;

  config.gpio_cfg.invert_flags.clk_inv =
    false;

  err = i2s_channel_init_pdm_rx_mode(
    micHandle,
    &config
  );

  if (err != ESP_OK) {
    stopMicrophone();
    return false;
  }

  err = i2s_channel_enable(micHandle);

  if (err != ESP_OK) {
    stopMicrophone();
    return false;
  }

  micReady = true;
  micDC = 0;

  Serial.println("MIC READY");

  return true;
}

// ====================================
// Speaker init - starts DISABLED
// ====================================

bool startSpeaker() {
  stopMicrophone();
  stopSpeaker();

  i2s_chan_config_t channelConfig =
    I2S_CHANNEL_DEFAULT_CONFIG(
      I2S_NUM_0,
      I2S_ROLE_MASTER
    );

  esp_err_t err = i2s_new_channel(
    &channelConfig,
    &speakerHandle,
    nullptr
  );

  if (err != ESP_OK) return false;

  i2s_std_config_t config = {};

  config.clk_cfg =
    I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_RATE);

  config.slot_cfg =
    I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
      I2S_DATA_BIT_WIDTH_16BIT,
      I2S_SLOT_MODE_STEREO
    );

  config.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  config.gpio_cfg.bclk = SPK_BCLK;
  config.gpio_cfg.ws = SPK_LRCK;
  config.gpio_cfg.dout = SPK_DATA;
  config.gpio_cfg.din = I2S_GPIO_UNUSED;

  err = i2s_channel_init_std_mode(
    speakerHandle,
    &config
  );

  if (err != ESP_OK) {
    stopSpeaker();
    return false;
  }

  speakerReady = true;
  speakerEnabled = false;

  Serial.println("SPEAKER READY (muted)");

  return true;
}

// ====================================
// Speaker mute / unmute
// ====================================

void muteSpeaker() {
  if (speakerEnabled && speakerHandle) {
    i2s_channel_disable(speakerHandle);
    speakerEnabled = false;
  }

  clearQueue();
  ledReady();
}

bool unmuteSpeaker() {
  if (!speakerReady || !speakerHandle) {
    return false;
  }

  if (!speakerEnabled) {
    esp_err_t err =
      i2s_channel_enable(speakerHandle);

    if (err != ESP_OK) {
      Serial.printf(
        "Speaker enable error: %s\n",
        esp_err_to_name(err)
      );
      return false;
    }

    speakerEnabled = true;
  }

  return true;
}

// ====================================
// Speaker write
// ====================================

void playAudio(const int16_t *audio) {
  if (!unmuteSpeaker()) return;

  int16_t stereo[FRAME_SAMPLES * 2];

  for (int i = 0; i < FRAME_SAMPLES; i++) {
    stereo[i * 2] = audio[i];
    stereo[i * 2 + 1] = audio[i];
  }

  size_t written = 0;

  i2s_channel_write(
    speakerHandle,
    stereo,
    sizeof(stereo),
    &written,
    100
  );
}

// ====================================
// Send microphone audio
// ====================================

int16_t clampSample(float value) {
  if (value > 32767) return 32767;
  if (value < -32768) return -32768;
  return (int16_t)value;
}

void sendAudio() {
  if (!micReady) return;

  int16_t raw[160];
  size_t bytesRead = 0;

  esp_err_t err = i2s_channel_read(
    micHandle,
    raw,
    sizeof(raw),
    &bytesRead,
    100
  );

  if (err != ESP_OK ||
      bytesRead != sizeof(raw)) {
    return;
  }

  AudioPacket packet = {};
  packet.sequence = sequenceNumber++;

  for (int i = 0; i < FRAME_SAMPLES; i++) {
    float input =
      ((float)raw[i * 2] +
       (float)raw[i * 2 + 1]) * 0.5f;

    micDC += 0.001f * (input - micDC);

    packet.audio[i] =
      clampSample((input - micDC) * MIC_GAIN);
  }

  err = esp_now_send(
    broadcastMac,
    (const uint8_t *)&packet,
    sizeof(packet)
  );

  if (err == ESP_OK) txPackets++;
}

// ====================================
// Wireless setup
// ====================================

bool setupWireless() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  esp_err_t err = esp_wifi_set_channel(
    WIFI_CHANNEL,
    WIFI_SECOND_CHAN_NONE
  );

  if (err != ESP_OK) return false;

  err = esp_now_init();
  if (err != ESP_OK) return false;

  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peer = {};

  memcpy(
    peer.peer_addr,
    broadcastMac,
    6
  );

  peer.channel = WIFI_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;

  err = esp_now_add_peer(&peer);

  if (err != ESP_OK) return false;

  Serial.println("ESP-NOW READY");
  return true;
}

// ====================================
// Setup
// ====================================

void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(BUTTON_PIN, INPUT);

  led.begin();
  led.setBrightness(80);
  ledReady();

  Serial.println("ATOM Echo Walkie-Talkie");
  Serial.println("Simple audio / auto mute");

  if (!setupWireless()) {
    Serial.println("Wireless failed");
    while (true) delay(1000);
  }

  if (!startSpeaker()) {
    Serial.println("Speaker failed");
    while (true) delay(1000);
  }

  rawButtonPrevious =
    digitalRead(BUTTON_PIN) == LOW;

  Serial.println("READY");
}

// ====================================
// Main loop
// ====================================

void loop() {
  bool rawPressed =
    digitalRead(BUTTON_PIN) == LOW;

  if (rawPressed != rawButtonPrevious) {
    rawButtonPrevious = rawPressed;
    buttonChangeTime = millis();
  }

  if (
    millis() - buttonChangeTime >= 25 &&
    rawPressed != buttonPressed
  ) {
    buttonPressed = rawPressed;

    if (buttonPressed) {
      transmitting = true;
      clearQueue();
      ledTransmit();

      if (!startMicrophone()) {
        Serial.println("MIC FAILED");
      } else {
        Serial.println("TX START");
      }

    } else {
      stopMicrophone();

      if (!startSpeaker()) {
        Serial.println("SPEAKER FAILED");
      }

      clearQueue();
      transmitting = false;
      ledReady();

      Serial.println("RX READY");
    }
  }

  if (buttonPressed) {
    sendAudio();

  } else {
    uint32_t lastRx;

    portENTER_CRITICAL(&queueMux);
    lastRx = lastPacketReceived;
    portEXIT_CRITICAL(&queueMux);

    // No incoming packets -> mute and clear.
    if (
      speakerEnabled &&
      millis() - lastRx > AUDIO_TIMEOUT_MS
    ) {
      muteSpeaker();
    }

    AudioPacket packet;

    if (popPacket(packet)) {
      ledReceive();
      playAudio(packet.audio);

    } else {
      delay(1);
    }
  }

  if (millis() - lastStatsTime >= 5000) {
    uint32_t rx;

    portENTER_CRITICAL(&queueMux);
    rx = rxPackets;
    portEXIT_CRITICAL(&queueMux);

    Serial.printf(
      "TX=%lu RX=%lu\n",
      (unsigned long)txPackets,
      (unsigned long)rx
    );

    lastStatsTime = millis();
  }
}
