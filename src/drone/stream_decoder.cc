#include "stream_decoder.h"

extern "C" {
#include <libARSAL/ARSAL.h>
#include <libavcodec/avcodec.h>
#include <libavutil/mem.h>
#include <libswscale/swscale.h>
}

#include <stdint.h>
#include <inttypes.h>

#if LIBAVCODEC_VERSION_INT < AV_VERSION_INT(55, 28, 1)
#define av_frame_alloc avcodec_alloc_frame
#define av_frame_free avcodec_free_frame
#endif

static const char* TAG = "StreamDecoder";

StreamDecoder::StreamDecoder() {}

StreamDecoder::~StreamDecoder() {
  Shutdown();

  if (ctx_) {
    avcodec_close(ctx_);
    delete[] ctx_->extradata;
    av_free(ctx_);
  }

  if (dropped_frame_) {
    av_frame_free(&dropped_frame_);
  }

  if (frames_) {
    for (size_t i = 0; i < frame_capacity_; i++) {
      av_frame_free(&frames_[i]);
    }

    delete[] frames_;
  }
}

bool StreamDecoder::Initialize(size_t max_frames, const uint8_t* sps_buffer,
                               size_t sps_buffer_size,
                               const uint8_t* pps_buffer,
                               size_t pps_buffer_size) {
  static bool libav_initialized = false;
  if (!libav_initialized) {
    avcodec_register_all();
  }

  av_log_set_level(AV_LOG_ERROR);
  auto codec = avcodec_find_decoder(AV_CODEC_ID_H264);
  if (!codec) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to find libav H264 codec!");
    return false;
  }

  ARSAL_Sem_Init(&frame_available_, 0, 0);
  ARSAL_Sem_Init(&frame_remaining_, 0, int(max_frames));

  frame_capacity_ = max_frames;
  frames_ = new AVFrame*[frame_capacity_];
  if (!frames_) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to allocate frames array.");
    return false;
  }

  dropped_frame_ = av_frame_alloc();
  for (size_t i = 0; i < frame_capacity_; i++) {
    AVFrame* frame = av_frame_alloc();
    if (!frame) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to allocate frame %" PRId64,
                  i);
      return false;
    }

    frames_[i] = frame;
  }

  ctx_ = avcodec_alloc_context3(codec);

  // Allocate the extradata
  ctx_->extradata = new uint8_t[sps_buffer_size + pps_buffer_size +
                                FF_INPUT_BUFFER_PADDING_SIZE];
  uint8_t* ptr = ctx_->extradata;
  memcpy(ptr, sps_buffer, sps_buffer_size);
  ptr += sps_buffer_size;
  memcpy(ptr, pps_buffer, pps_buffer_size);
  ptr += pps_buffer_size;
  memset(ptr, 0, FF_INPUT_BUFFER_PADDING_SIZE);

  ctx_->extradata_size =
      int(sps_buffer_size + pps_buffer_size + FF_INPUT_BUFFER_PADDING_SIZE);

  ctx_->pix_fmt = AV_PIX_FMT_YUV420P;

  if (avcodec_open2(ctx_, codec, nullptr) < 0) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to open the H264 codec");
    av_free(ctx_);
    return false;
  }

  // Create a default filter for the converter.
  filter_ = sws_getDefaultFilter(0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0);

  return true;
}

void StreamDecoder::Shutdown() {
  // Releases the consumer thread from GetFrame.
  shutdown_requested_ = true;
  ARSAL_Sem_Post(&frame_available_);
}

int StreamDecoder::ParseData(const uint8_t* data, size_t size) {
  AVPacket packet;
  av_init_packet(&packet);
  packet.size = static_cast<int>(size);
  packet.data = const_cast<uint8_t*>(data);

  int ret = avcodec_send_packet(ctx_, &packet);
  if (ret < 0) {
    return ret;
  }

  // Acquire a free frame.
  timespec timeval;
  timeval.tv_sec = 0;
  timeval.tv_nsec = 0;
  ret = ARSAL_Sem_Timedwait(&frame_remaining_, &timeval);
  if (ret < 0) {
    // No free frames.
    avcodec_receive_frame(ctx_, dropped_frame_);
    av_frame_unref(dropped_frame_);
    return 1;
  }

  AVFrame* frame = frames_[frame_free_];
  if (frame) {
    ret = avcodec_receive_frame(ctx_, frame);
    if (ret == 0) {
      // Frame received. Mark as used and signal the semaphores.
      frame_free_ = (frame_free_ + 1) % frame_capacity_;
      ARSAL_Sem_Post(&frame_available_);
    }
  }

  return 0;
}

const uint32_t StreamDecoder::GetWidth() { return ctx_->width; }
const uint32_t StreamDecoder::GetHeight() { return ctx_->height; }

int StreamDecoder::GetFrame(DecodedFrame_t* decoded_frame) {
  if (ARSAL_Sem_Wait(&frame_available_) < 0) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                "Failed to wait on frame_available_ semaphore.");
    return -1;
  }

  if (shutdown_requested_) {
    return 1;
  }

  AVFrame* frame = frames_[frame_cur_];
  frame_cur_ = (frame_cur_ + 1) % frame_capacity_;

  sws_ = sws_getCachedContext(sws_, frame->width, frame->height, ctx_->pix_fmt,
                              frame->width, frame->height, AV_PIX_FMT_RGB24, 0,
                              filter_, filter_, nullptr);
  if (!sws_) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                "sws_getCachedContext returned nullptr");
    return -1;
  }

  if (decoded_frame->data != nullptr) {
    if (decoded_frame->data_capacity <
        size_t(frame->width * frame->height) * 3ull) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Insufficient space in user buffer.");
      return -2;
    }
  } else {
    // Allocate a RGB buffer.
    decoded_frame->data = new uint8_t[frame->width * frame->height * 3];
    decoded_frame->data_capacity = frame->width * frame->height * 3;
  }

  // Now convert the frame into the user buffer.
  int dstStride = ctx_->width * 3;
  sws_scale(sws_, frame->data, frame->linesize, 0, frame->height,
            &decoded_frame->data, &dstStride);

  decoded_frame->width = frame->width;
  decoded_frame->height = frame->height;
  decoded_frame->data_size = frame->width * frame->height * 3;
  decoded_frame->line_stride = dstStride;

  // After posting this semaphore, the frame is free for the decoder to use.
  av_frame_unref(frame);
  ARSAL_Sem_Post(&frame_remaining_);

  return 0;
}

void StreamDecoder::FlushData() { avcodec_flush_buffers(ctx_); }