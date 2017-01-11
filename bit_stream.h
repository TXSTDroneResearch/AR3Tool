#ifndef _BIT_STREAM_H_
#define _BIT_STREAM_H_

#include <cstddef>
#include <cstdint>

class BitStream {
 public:
  BitStream(uint8_t* buffer, size_t size_in_bits);
  ~BitStream();

  const uint8_t* buffer() const { return buffer_; }
  uint8_t* buffer() { return buffer_; }
  size_t offset_bits() const { return offset_bits_; }
  size_t size_bits() const { return size_bits_; }

  void Advance(size_t num_bits);
  void SetOffset(size_t offset_bits);
  size_t BitsRemaining();

  // Note: num_bits MUST be in the range 0-57 (inclusive)
  uint64_t Peek(size_t num_bits);
  uint64_t Read(size_t num_bits);
  bool Write(uint64_t val, size_t num_bits);  // TODO(DrChat): Not tested!

  size_t Copy(uint8_t* dest_buffer, size_t num_bits);

 private:
  uint8_t* buffer_ = nullptr;
  size_t offset_bits_ = 0;
  size_t size_bits_ = 0;
};

#endif  // _BIT_STREAM_H_
