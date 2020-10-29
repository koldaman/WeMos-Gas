/**************************************************************
  Signalizace blikanim LED;
 **************************************************************/
#include "Arduino.h"

#include "Blink.h"
#include <Ticker.h>
#include <vector>

Blink::Blink(int pinNumber) {
   _pinNumber = pinNumber;
   _inverse = false;
}

Blink::Blink(int pinNumber, std::vector<int> intervals, int repetitionCount) {
   _pinNumber = pinNumber;
   init(intervals, repetitionCount);
}

void Blink::init(std::vector<int> intervals, int repetitionCount) {
   _intervals = intervals;
   _currentInterval = 0;
   _repetitionCount = repetitionCount;
   _count = 0;
   _stopRequested = false;
   _state = _inverse ? HIGH : LOW;
   pinMode(_pinNumber, OUTPUT);
}

void Blink::inverse(bool inverse) {
   _inverse = inverse;
}

void Blink::changeState(Blink *blink) {
   if (blink->_stopRequested || (blink->_repetitionCount > 0 && blink->_count == blink->_repetitionCount)) {
      blink->stop();
      return;
   }

   blink->_state = blink->_state == LOW ? HIGH : LOW;

   digitalWrite(blink->_pinNumber, blink->_inverse ? (blink->_state == LOW ? HIGH : LOW) : blink->_state);

   blink->_ticker.attach_ms(blink->_intervals[blink->_currentInterval], &Blink::changeState, blink);

   if (blink->_currentInterval == blink->_intervals.size()-1) {
      blink->_currentInterval = 0;
      blink->_count++;
   } else {
      blink->_currentInterval++;
   }
}

void Blink::stop() {
   _ticker.detach();
   _state = _inverse ? HIGH : LOW;
   _stopRequested = true;
   digitalWrite(_pinNumber, _state);
}

void Blink::start() {
   stop();
   _stopRequested = false;
   _count = 0;
   _currentInterval = 0;
   Blink::changeState(this);
}
