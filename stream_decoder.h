#ifndef _STREAM_DECODER_H_
#define _STREAM_DECODER_H_

#include <cstdint>
#include <cstdlib>

class AVCodecContext;
class AVFrame;
struct SwsContext;
struct SwsFilter;
typedef void* ARSAL_Sem_t;

struct StreamParameters_t {
  uint32_t width;
  uint32_t height;
};

struct DecodedFrame_t {
  uint32_t width;      // (out) width of the frame
  uint32_t height;     // (out) height of the frame
  uint64_t recv_time;  // (out) time the frame was received

  uint8_t* data;         // (in/out) data buffer in RGB24 format
                         // if nullptr, data will be allocated for you.
  size_t data_capacity;  // (in/out) data capacity
  size_t data_size;      // (out) filled data
  int line_stride;       // (out) Line width in bytes
};

/**
  * StreamDecoder class
  * Decodes streams! Yay!
  *
  * One thread can send data into the decoder, and another can dequeue frames.
  * Or one thread can do both, who knows?
  */
class StreamDecoder {
 public:
  StreamDecoder();
  ~StreamDecoder();

  // Initialize the stream decoder.
  bool Initialize(size_t max_frames, const uint8_t* sps_buffer,
                  size_t sps_buffer_size, const uint8_t* pps_buffer,
                  size_t pps_buffer_size);

  void Shutdown();

  // Parse new data. Returns 0 on success, negative value on failure, 1 if too
  // many frames queued.
  int ParseData(const uint8_t* data, size_t size);

  const uint32_t GetWidth();
  const uint32_t GetHeight();

  // Retrieves a successfully decoded frame into decoded_frame. Returns 0 on
  // success, a negative value on failure, or a positive value on shutdown.
  int GetFrame(DecodedFrame_t* decoded_frame);

  // Flush previous data
  void FlushData();

 private:
  AVCodecContext* ctx_ = nullptr;

  SwsContext* sws_ = nullptr;
  SwsFilter* filter_ = nullptr;
  uint8_t* converted_data_ = nullptr;
  bool shutdown_requested_ = false;

  AVFrame* dropped_frame_ = nullptr;
  AVFrame** frames_ = nullptr;  // Rolling buffer of frames.
  size_t frame_capacity_ = 0;   // Maximum number of frames.
  size_t frame_cur_ = 0;        // First frame to be dequeued.
  size_t frame_free_ = 0;       // First free frame.
  ARSAL_Sem_t frame_available_ = nullptr;
  ARSAL_Sem_t frame_remaining_ = nullptr;
};

#endif  // _STREAM_DECODER_H_