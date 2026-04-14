#ifndef IBUS_h
#define IBUS_h

#include "Arduino.h"
#include "elapsedMillis.h"

class IBUS {
  public:
    IBUS(HardwareSerial& bus);
    void begin();
    bool read(uint16_t* channels, bool* failsafe, bool* lostFrame);
    void setEndPoints(uint8_t channel, uint16_t min, uint16_t max);
    void getEndPoints(uint8_t channel, uint16_t* min, uint16_t* max);
    void setReadCal(uint8_t ch, float* coeff, uint8_t len);
    void getReadCal(uint8_t ch, float* coeff, uint8_t len);
    ~IBUS();

  private:
    const uint32_t _ibusBaud    = 115200;
    static const uint8_t _numChannels = 14;
    static const uint8_t _frameSize   = 32;

    HardwareSerial* _bus;
    uint8_t _buffer[_frameSize];

    // Timeout tracks time since last VALID frame — not last byte.
    // i-BUS has no native failsafe flag in the packet (unlike SBUS).
    // Failsafe is inferred from frame timeout only.
    // At 115200 baud a full 32-byte frame takes ~2.8ms.
    // Two packets per cycle at 7ms → timeout of 20ms covers
    // 2+ missed cycles before declaring signal lost.
    const uint32_t IBUS_TIMEOUT_US = 20000;
    elapsedMicros _ibusTime;

    uint8_t _parserState = 0;
    uint8_t _curByte;

    // Calibration
    const uint16_t _defaultMin = 1000;
    const uint16_t _defaultMax = 2000;
    uint16_t _ibusMin[_numChannels];
    uint16_t _ibusMax[_numChannels];
    float    _ibusScale[_numChannels];
    float    _ibusBias[_numChannels];
    float**  _readCoeff;
    uint8_t  _readLen[_numChannels];
    bool     _useReadCoeff[_numChannels];

    void  scaleBias(uint8_t channel);
    float PolyVal(size_t PolySize, float* Coefficients, float X);
};

#endif
