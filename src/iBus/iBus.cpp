#include "IBus.h"

IBUS::IBUS(HardwareSerial& bus) : _ibusTime(0) {
  _bus = &bus;
  for (uint8_t i = 0; i < _numChannels; i++) {
    _readLen[i]      = 0;
    _useReadCoeff[i] = false;
  }
  _readCoeff = nullptr;
}

void IBUS::begin() {
  _bus->begin(_ibusBaud, SERIAL_8N1);
  for (uint8_t i = 0; i < _numChannels; i++) {
    setEndPoints(i, _defaultMin, _defaultMax);
  }
}

bool IBUS::read(uint16_t* channels, bool* failsafe, bool* lostFrame) {

  // FIX 4: Timeout tracks time since last VALID frame, not last byte.
  // Moved _ibusTime reset into the valid frame branch below.
  if (_ibusTime > IBUS_TIMEOUT_US) {
    _parserState = 0;
    if (lostFrame) *lostFrame = true;
    if (failsafe)  *failsafe  = true;
  }

  while (_bus->available() > 0) {
    _curByte = _bus->read();

    if (_parserState == 0) {
      // Wait for first header byte 0x20
      if (_curByte == 0x20) {
        _buffer[0] = _curByte;
        _parserState = 1;
      }

    } else if (_parserState == 1) {
      // FIX 2: Validate second header byte 0x40 before accepting frame.
      // A stray 0x20 data byte in a corrupted frame could otherwise
      // trigger a false resync and misalign the parser on the next
      // valid frame. Confirming 0x40 here eliminates that risk.
      if (_curByte == 0x40) {
        _buffer[1] = _curByte;
        _parserState = 2;
      } else if (_curByte == 0x20) {
        // New 0x20 while waiting for 0x40 — treat as new frame start
        _buffer[0] = _curByte;
        _parserState = 1;
      } else {
        // Not a valid second header byte — false start, reset
        _parserState = 0;
      }

    } else {
      // FIX 3: Buffer indexing — store at _parserState (not _parserState-1).
      // Previous off-by-one caused the second byte to overwrite the
      // first header byte (0x20) in the buffer, corrupting every frame.
      _buffer[_parserState] = _curByte;
      _parserState++;

      if (_parserState == _frameSize) {
        // Full frame received — validate checksum
        uint16_t checksum = 0xFFFF;
        for (uint8_t i = 0; i < _frameSize - 2; i++) {
          checksum -= _buffer[i];
        }
        uint16_t rxChecksum = _buffer[_frameSize - 2]
                            | (_buffer[_frameSize - 1] << 8);

        if (checksum == rxChecksum) {
          // FIX 4: Reset timeout only on a valid frame
          _ibusTime = 0;

          if (channels) {
            for (uint8_t ch = 0; ch < _numChannels; ch++) {
              // FIX 1: Raw value extracted first (1000-2000 range)
              uint16_t rawValue = _buffer[2 + ch * 2]
                                | (_buffer[3 + ch * 2] << 8);

              if (_useReadCoeff[ch]) {
                // Polynomial calibration path — only used if explicitly set
                float calibratedValue = rawValue * _ibusScale[ch]
                                      + _ibusBias[ch];
                calibratedValue = PolyVal(_readLen[ch],
                                          _readCoeff[ch],
                                          calibratedValue);
                channels[ch] = (uint16_t)calibratedValue;
              } else {
                // FIX 1: Default path outputs raw value directly (1000-2000).
                // Previous scaleBias() mapped to -1.0 to +1.0 normalized
                // range which is incompatible with the FCU channel pipeline
                // expecting 1000-2000 µs equivalent values.
                // Unity pass-through is correct for standard i-BUS use.
                channels[ch] = rawValue;
              }
            }
          }

          if (failsafe)  *failsafe  = false;
          if (lostFrame) *lostFrame = false;
          _parserState = 0;
          return true;

        } else {
          // Checksum mismatch — discard frame
          _parserState = 0;
          if (lostFrame) *lostFrame = true;
          return false;
        }
      }
    }
  }
  return false;
}

IBUS::~IBUS() {
  if (_readCoeff) {
    for (uint8_t i = 0; i < _numChannels; i++) {
      if (_readCoeff[i]) free(_readCoeff[i]);
    }
    free(_readCoeff);
  }
}

void IBUS::setEndPoints(uint8_t channel, uint16_t min, uint16_t max) {
  _ibusMin[channel] = min;
  _ibusMax[channel] = max;
  scaleBias(channel);
}

void IBUS::getEndPoints(uint8_t channel, uint16_t* min, uint16_t* max) {
  if (min && max) {
    *min = _ibusMin[channel];
    *max = _ibusMax[channel];
  }
}

void IBUS::setReadCal(uint8_t channel, float* coeff, uint8_t len) {
  if (!coeff) return;

  // FIX 5: Zero-initialize the pointer array after malloc.
  // Previously freshly malloc'd memory was not zeroed — the
  // !_readCoeff[channel] check was undefined behavior on
  // uninitialized pointers.
  if (!_readCoeff) {
    _readCoeff = (float**) malloc(sizeof(float*) * _numChannels);
    memset(_readCoeff, 0, sizeof(float*) * _numChannels);
  }

  if (_readCoeff[channel]) free(_readCoeff[channel]);
  _readCoeff[channel] = (float*) malloc(sizeof(float) * len);

  for (uint8_t i = 0; i < len; i++) {
    _readCoeff[channel][i] = coeff[i];
  }
  _readLen[channel]      = len;
  _useReadCoeff[channel] = true;
}

void IBUS::getReadCal(uint8_t channel, float* coeff, uint8_t len) {
  if (!coeff) return;
  for (uint8_t i = 0; i < _readLen[channel] && i < len; i++) {
    coeff[i] = _readCoeff[channel][i];
  }
}

void IBUS::scaleBias(uint8_t channel) {
  // Scale and bias are kept for polynomial calibration use only.
  // The default read() path bypasses these and outputs raw values
  // directly in 1000-2000 range. These are only applied when
  // setReadCal() has been called for a channel.
  _ibusScale[channel] = 2.0f / ((float)_ibusMax[channel]
                               - (float)_ibusMin[channel]);
  _ibusBias[channel]  = -1.0f * ((float)_ibusMin[channel]
                       + ((float)_ibusMax[channel]
                       -  (float)_ibusMin[channel]) / 2.0f)
                       * _ibusScale[channel];
}

float IBUS::PolyVal(size_t PolySize, float* Coefficients, float X) {
  if (!Coefficients) return 0;
  float Y = Coefficients[0];
  for (uint8_t i = 1; i < PolySize; i++) {
    Y = Y * X + Coefficients[i];
  }
  return Y;
}
