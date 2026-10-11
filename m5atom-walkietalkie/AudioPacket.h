
#pragma once
#include <stdint.h>

#define FRAME_SAMPLES 80

struct __attribute__((packed)) AudioPacket {
  uint16_t sequence;
  int16_t audio[FRAME_SAMPLES];
};
