#ifndef sistem_h
#define sistem_h

#include "ads.h"
#include "ledController.h"

enum ChannelState {
  CHANNEL_IDLE,
  LED_ON,
  DELAY_COUNTING,
  ADC_READING_PHASE,
  CYCLE_COMPLETE
};

struct ChannelData {
  ChannelState state;
  unsigned long startTime;
  unsigned long adcAccumulator;
  int adcReadCount;
  bool processComplete;
};

struct sistem {
  String SisteminAdi;
  String versiyon;
  ads1115 myAds;
  ledController myLeds;
  ChannelData channels[4];
  int currentChannel;
  bool systemEnabled;
  volatile bool timerExpired;
  String rxBuffer;
  bool jsonCallback;
};

#endif