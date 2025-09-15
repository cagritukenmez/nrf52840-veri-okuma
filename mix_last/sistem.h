#ifndef sistem_h
#define sistem_h

#include "ads.h"
#include "ledController.h"

enum ChannelState {
  CHANNEL_IDLE,
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
  volatile bool timer1Expired;
  volatile bool timer2Expired;
  String rxBuffer;
  bool jsonCallback;
};

#endif