#ifndef _STREAM_DISPLAY_H_
#define _STREAM_DISPLAY_H_

#include <cinttypes>
#include <csignal>
#include <cstdio>

class StreamDisplay {
 public:
  StreamDisplay();
  ~StreamDisplay();

  bool Initialize(uint32_t width, uint32_t height);
  void Shutdown();

  // Sends raw data to the player.
  void DisplayFrame(const uint8_t* data, size_t length);

 private:
  pid_t player_pid_ = 0;
  char* player_fifo_name_ = nullptr;
  FILE* player_fifo_ = nullptr;

  uint32_t width_ = 0;
  uint32_t height_ = 0;
};

#endif  // _STREAM_DISPLAY_H_