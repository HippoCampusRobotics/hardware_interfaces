#include "packet.hpp"

#include "cobs.hpp"

bool Packet::AddByte(const uint8_t _byte) {
  if (!complete_ && (write_pointer_ < buffer_end_)) {
    if (_byte == kDelimiter) {
      complete_ = true;
    }
    *write_pointer_++ = _byte;
    ++size_;
    return true;
  }
  return false;
}

bool Packet::SetPayload(const uint8_t *_payload, int _length) {
  write_pointer_ = buffer_ + kPayloadOffset;
  if (_length > kBufferSize - kTotalOverhead) {
    return false;
  }
  for (int i = 0; i < _length; ++i) {
    *write_pointer_++ = *_payload++;
  }
  size_ = _length + kTotalOverhead;
  return true;
}

void Packet::Packetize() {
  WriteCrc();
  cobs_encode(buffer_, Size());
}

bool Packet::Decode() {
  uint8_t *data = cobs_decode(buffer_, Size());
  if (!data) {
    return false;
  }
  return true;
}

bool Packet::CrcOk() {
  uint32_t crc = ReadCrc();
  crc_engine_.reset();
  crc_engine_.process_bytes(buffer_ + kPayloadOffset, PayloadSize());
  uint32_t crc_expected = crc_engine_.checksum();
  return crc == crc_expected;
}

void Packet::WriteCrc() {
  crc_engine_.reset();
  crc_engine_.process_bytes(buffer_ + kPayloadOffset, PayloadSize());
  uint32_t crc = crc_engine_.checksum();
  uint8_t *writer = buffer_ + kPayloadOffset + PayloadSize();
  *writer++ = (uint8_t)((crc >> 24) & 0xFF);
  *writer++ = (uint8_t)((crc >> 16) & 0xFF);
  *writer++ = (uint8_t)((crc >> 8) & 0xFF);
  *writer++ = (uint8_t)(crc & 0xFF);
}


void Packet::Reset() {
  complete_ = false;
  write_pointer_ = buffer_;
  size_ = 0;
}
