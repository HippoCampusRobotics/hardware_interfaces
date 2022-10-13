#include "cobs.hpp"

void cobs_encode(uint8_t *_buffer, int _length) {
  _buffer[0] = 0;
  _buffer[_length - 1] = 0;
  uint8_t offset = 0;
  for (int i = 1; i < _length; ++i) {
    ++offset;
    if (_buffer[i] == 0) {
      _buffer[i - offset] = offset;
      offset = 0;
    }
  }
}

uint8_t *cobs_decode(uint8_t *_buffer, int _length) {
  if (!_buffer) {
    return nullptr;
  }
  int i = 0;
  int next_zero = _buffer[0];
  while (true) {
    i += next_zero;
    if (i >= _length) {
      return nullptr;
    }
    next_zero = _buffer[i];
    // delimiter found
    if (next_zero == 0) {
      return &_buffer[1];
    }
    _buffer[i] = 0;
  }
}
