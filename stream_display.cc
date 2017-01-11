#include "stream_display.h"

extern "C" {
#include <libARSAL/ARSAL.h>
}

#include <errno.h>
#include <signal.h>
#include <stdlib.h>

#define DISPLAY_FIFO_DIR "/tmp/arsdk_XXXXXX"  // FIFO dir pattern
#define DISPLAY_FIFO_NAME "ar_mplayer"        // FIFO name
#define TAG "StreamDisplay"

StreamDisplay::StreamDisplay() {}
StreamDisplay::~StreamDisplay() { Shutdown(); }

bool StreamDisplay::Initialize(uint32_t width, uint32_t height) {
  if (player_pid_ != 0) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                "Called StreamDisplay::Initialize twice!");
    return false;
  }

  // Set up a FIFO for mplayer.
  char fifo_dir[sizeof(DISPLAY_FIFO_DIR)];
  snprintf(fifo_dir, sizeof(fifo_dir), "%s", DISPLAY_FIFO_DIR);
  if (mkdtemp(fifo_dir) == nullptr) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                "Unable to create FIFO dir (errno: %d / %s)", errno,
                strerror(errno));
    return false;
  }

  player_fifo_name_ = new char[256];
  sprintf(player_fifo_name_, "%s/%s", fifo_dir, DISPLAY_FIFO_NAME);
  if (mkfifo(player_fifo_name_, 0666) < 0) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                "Unable to create FIFO (errno: %d / %s)", errno,
                strerror(errno));
    return false;
  }

  ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "Created FIFO at %s", player_fifo_name_);

  // Start mplayer if requested
  if (player_pid_ == 0) {
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "Launching mplayer");
    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "Width: %d Height: %d", width, height);
    if ((player_pid_ = fork()) == 0) {
      // In child process.
      // TODO(justin): Enum specifying data format
      char params[256];
      snprintf(params, sizeof(params), "w=%d:h=%d:format=rgb24", width, height);

      execlp("xterm", "xterm", "-e", "mplayer", player_fifo_name_, "-demuxer",
             "rawvideo", "-rawvideo", params, "-benchmark", /*"-really-quiet",*/
             NULL);

      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to launch mplayer");
      exit(1);
    }

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "Opening fifo %s", player_fifo_name_);
    player_fifo_ = fopen(player_fifo_name_, "wb");
  }

  return true;
}

void StreamDisplay::Shutdown() {
  if (player_pid_) {
    kill(player_pid_, SIGKILL);
  }
  if (player_fifo_) {
    fclose(player_fifo_);
  }
  if (player_fifo_name_) {
    remove(player_fifo_name_);
    // remove(fifo_dir);
    delete[] player_fifo_name_;
  }

  player_pid_ = 0;
  player_fifo_ = nullptr;
  player_fifo_name_ = nullptr;
  width_ = 0;
  height_ = 0;
}

void StreamDisplay::DisplayFrame(const uint8_t* data, size_t length) {
  fwrite(data, length, 1, player_fifo_);
}
