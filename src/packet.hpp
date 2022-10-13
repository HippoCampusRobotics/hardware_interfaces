#pragma once
#include <stdint.h>

#include <boost/crc.hpp>

class Packet {
 public:
  static constexpr uint8_t kDelimiter{0};
  bool AddByte(const uint8_t byte);
  void Packetize();
  bool Decode();
  bool CrcOk();
  bool SetPayload(const uint8_t* _payload, int _length);
  const uint8_t* Payload() const { return buffer_ + kPayloadOffset; }
  uint8_t* MutablePayload() { return buffer_ + kPayloadOffset; }
  int PayloadSize() const { return Size() - kTotalOverhead; }
  const uint8_t* Data() const { return buffer_; }
  int Size() const { return size_; }
  bool Complete() const { return complete_; }
  void Reset();

 private:
  static constexpr int kCobsOverhead = 2;
  static constexpr int kPayloadOffset = 1;
  static constexpr int kCrcOverhead = sizeof(uint32_t);
  static constexpr int kTotalOverhead = kCobsOverhead + kCrcOverhead;
  static constexpr int kBufferSize{256};
  void WriteCrc();
  inline uint32_t DecodeCrc(uint8_t* _buffer) {
    return (_buffer[0] << 24) | (_buffer[1] << 16) | (_buffer[2] << 8) |
           _buffer[3];
  }
  inline uint32_t ReadCrc() {
    return DecodeCrc(buffer_ + kPayloadOffset + PayloadSize());
  }
  uint8_t buffer_[kBufferSize];
  uint8_t* write_pointer_ = buffer_;
  const uint8_t* buffer_end_ = buffer_ + kBufferSize;
  int size_{0};
  bool complete_{false};
  boost::crc_32_type crc_engine_;
};
