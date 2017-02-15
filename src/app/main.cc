#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <cstring>

#include <vector>

extern "C" {
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARSAL/ARSAL.h>
}

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "drone/stream_decoder.h"

#define TAG "main.cc"

#define BEBOP_IP_ADDRESS "192.168.42.1"
#define BEBOP_DISCOVERY_PORT 44444

#define DISPLAY_OPENCV 1          // Display on an OpenCV window
#define DISPLAY_LOCAL_DECODING 1  // Decode in-app

enum ControlState {
  CONTROLSTATE_NONE = 0,
  CONTROLSTATE_COMMANDING,
  CONTROLSTATE_CALIBRATING,
};

struct CalibrationState_t {
  // lame way to request captures in calibration mode :|
  bool snap_requested;
  bool dump_requested;
  bool matgen_requested;

  std::vector<std::vector<cv::Vec2f>> image_points;
};

struct DeviceState_t {
  ARCONTROLLER_Device_t* controller;
  StreamDecoder* stream_decoder;
  ARSAL_Thread_t stream_thread;
  ARSAL_Sem_t state_sem;
  ARSAL_Sem_t frame_sem;

  ARSAL_Thread_t input_thread;

#if DISPLAY_OPENCV
  ARSAL_Thread_t display_thread;
#endif

  ControlState control_state;
  CalibrationState_t* calibration_state;

  uint8_t battery;
  float pitch, yaw, roll;  // yaw: [-pi, pi], pitch & roll: [-pi/2, pi/2]
  double latitude, longitude, altitude;
  double alt_hiprecision;

  float camera_fov;
  int8_t camera_pan, camera_tilt;
  float camera_pan_min, camera_pan_max;
  float camera_tilt_min, camera_tilt_max;
};

// Converts a given Euler angles to Rotation Matrix
cv::Mat ConvertEulerToMatrixYPR(const cv::Mat& euler) {
  double ca = cos(euler.at<double>(0));
  double sa = sin(euler.at<double>(0));
  double cb = cos(euler.at<double>(1));
  double sb = sin(euler.at<double>(1));
  double ch = cos(euler.at<double>(2));
  double sh = sin(euler.at<double>(2));

  // Yaw
  cv::Mat Rx = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, ch, -sh, 0, sh, ch);

  // Pitch
  cv::Mat Ry = (cv::Mat_<double>(3, 3) << cb, 0, sb, 0, 1, 0, -sb, 0, cb);

  // Roll
  cv::Mat Rz = (cv::Mat_<double>(3, 3) << ca, -sa, 0, sa, ca, 0, 0, 0, 1);

  // Rotation order: Yaw -> Pitch -> Roll
  return Rz * Ry * Rx;
}

cv::Point3f operator*(cv::Mat M, const cv::Point3f& p) {
  cv::Mat_<float> src(3, 1);
  src(0, 0) = p.x;
  src(1, 0) = p.y;
  src(2, 0) = p.z;

  cv::Mat_<float> dst = M * src;
  return cv::Point3f(dst(0, 0), dst(1, 0), dst(2, 0));
}

template <typename T>
T deg2rad(T deg) {
  return deg * CV_PI / 180.0;
}

#if DISPLAY_OPENCV
void* DisplayThread(void* param) {
  DeviceState_t* device_state = reinterpret_cast<DeviceState_t*>(param);
  StreamDecoder* stream_decoder = device_state->stream_decoder;
  cv::namedWindow("Stream Display");

  DecodedFrame_t frame;
  std::memset(&frame, 0, sizeof(frame));
  while (true) {
    int ret = stream_decoder->GetFrame(&frame);
    if (ret > 0) {
      break;
    }

    if (ret == 0) {
      auto width = frame.width;
      auto height = frame.height;

      cv::Mat mat(height, width, CV_8UC3, frame.data);
      if (device_state->control_state == CONTROLSTATE_COMMANDING) {
        auto stats_loc =
            cv::format("GPS: %f %f %f", device_state->latitude,
                       device_state->longitude, device_state->altitude);
        auto stats_rot = cv::format("PYR: %f %f %f", device_state->pitch,
                                    device_state->yaw, device_state->roll);
        auto stats_cam = cv::format("CAM: %d %d", device_state->camera_tilt,
                                    device_state->camera_pan);
        auto stats_alt = cv::format("ALT: %f", device_state->alt_hiprecision);
        auto stats_bat = cv::format("BAT: %d%%", device_state->battery);

        // Manipulation: OpenCV makes it ez!
        // OpenCV does not make a copy of frame.data, it operates on it
        // directly.
        cv::putText(mat, stats_loc, cv::Point(20, 20),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));
        cv::putText(mat, stats_rot, cv::Point(20, 40),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));
        cv::putText(mat, stats_cam, cv::Point(20, 60),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));
        cv::putText(mat, stats_alt, cv::Point(20, 80),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));
        cv::putText(mat, stats_bat, cv::Point(20, 100),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));

        // Calibration matrix: Obtained from taking pictures of a checkerboard
        // in calibration mode.
        // TODO(justin): Unhardcode this!
        // [fx, 0, cx]
        // [0, fy, cy]
        // [0,  0,  1]
        // 515.752791 0.000000 428.777745
        // 0.000000 506.591215 243.577033
        // 0.000000 0.000000 1.000000
        double _cm[9] = {515.752791, 0, 428, 0, 506.591215, 240, 0, 0, 1};
        cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, _cm);

        cv::Mat rot_matrix = ConvertEulerToMatrixYPR(
            cv::Mat(cv::Vec3d(device_state->yaw, 0, 0)));
        cv::Vec3d rvec;
        cv::Rodrigues(rot_matrix, rvec);

        std::vector<cv::Point3f> points3d(4);
        points3d[3] = {5, 0, 0};  // origin
        points3d[0] = points3d[3] + cv::Point3f(1, 0, 0);
        points3d[1] = points3d[3] + cv::Point3f(0, 1, 0);
        points3d[2] = points3d[3] + cv::Point3f(0, 0, 1);

        // TODO(justin): projectPoints does not do culling. Points behind the
        // camera are still projected onscreen (up-side-down)
        std::vector<cv::Point2f> points2d(4);
        cv::Vec3d tvec(0, 0, 0);
        cv::Vec4d dcoeffs(0, 0, 0, 0);
        cv::projectPoints(points3d, rvec, tvec, cam_matrix, dcoeffs, points2d);

        for (int i = 0; i < 3; i++) {
          cv::Scalar col(0);
          col[i] = 255;
          cv::line(mat, points2d[3], points2d[i], col);
        }
      } else if (device_state->control_state == CONTROLSTATE_CALIBRATING) {
        CalibrationState_t* calibration_state = device_state->calibration_state;

        cv::putText(mat, "press 'c' to swap to control mode", cv::Point(20, 20),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));
        cv::putText(
            mat,
            cv::format("press 'p' to capture the current image (%d so far...)",
                       calibration_state->image_points.size()),
            cv::Point(20, 40), cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
            cv::Scalar(255, 255, 255));
        cv::putText(mat, "press 'd' to dump all captured data",
                    cv::Point(20, 60), cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                    1.f, cv::Scalar(255, 255, 255));
        cv::putText(mat, "press 'l' to generate an intrinsic matrix",
                    cv::Point(20, 80), cv::HersheyFonts::FONT_HERSHEY_PLAIN,
                    1.f, cv::Scalar(255, 255, 255));

        if (!calibration_state) {
          ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                      "calibration_state is NULL while in calibration mode!");
          // continue (and crash)
        }

        // Use the last stream image as a 'snapshot' to calibrate the camera.
        cv::Mat grayscale;
        cv::cvtColor(mat, grayscale, cv::COLOR_RGB2GRAY);

        // TODO(justin): We should really be telling the drone to take a picture
        // and calibrating based off of that, but I'm too lazy :)
        cv::Size board_size(6, 9);
        std::vector<cv::Vec2f> image_points;
        bool found =
            cv::findChessboardCorners(grayscale, board_size, image_points);
        if (found) {
          cv::putText(mat, "found checkerboard", cv::Point(20, 100),
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                      cv::Scalar(0, 255, 0));
          cv::drawChessboardCorners(mat, board_size, image_points, found);

          if (calibration_state->snap_requested) {
            // Store in array.
            ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Snapshotting checkerboard...");
            calibration_state->image_points.push_back(image_points);
          }
        } else {
          cv::putText(mat, "failed to find checkerboard", cv::Point(20, 100),
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                      cv::Scalar(255, 0, 0));

          if (calibration_state->snap_requested) {
            // draw a big fat redtangle
            cv::rectangle(mat, cv::Point(0, 120), cv::Point(width, height),
                          cv::Scalar(255, 0, 0));
          }
        }

        if (calibration_state->matgen_requested &&
            calibration_state->image_points.size() > 0) {
          ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Generating matrix...");

          // calibrate camera.
          cv::Size image_size(width, height);

          // build object points (the base, CV will find camera pos from this)
          // Note: If we actually calculate the base position, rvec and tvec
          // won't be useless!
          std::vector<cv::Vec3f> obj_corners;
          float square_size = 1.f;  // in metres(?)
          for (int i = 0; i < board_size.height; i++) {
            for (int j = 0; j < board_size.width; j++) {
              obj_corners.push_back(
                  cv::Vec3f(j * square_size, i * square_size, 0));
            }
          }

          std::vector<std::vector<cv::Vec3f>> obj_points;
          obj_points.push_back(obj_corners);
          // Duplicate the first element.
          obj_points.resize(calibration_state->image_points.size(),
                            obj_points[0]);

          // Note: All parameters come back as 64f regardless of input format.
          cv::Mat cam_matrix = cv::Mat::eye(3, 3, CV_64F);
          cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
          std::vector<cv::Mat> rvecs, tvecs;
          rvecs.resize(obj_points.size(), cv::Mat(3, 1, CV_64F));
          tvecs.resize(obj_points.size(), cv::Mat(3, 1, CV_64F));

          double rms = cv::calibrateCamera(
              obj_points, calibration_state->image_points, image_size,
              cam_matrix, dist_coeffs, rvecs, tvecs);
          ARSAL_PRINT(ARSAL_PRINT_INFO, TAG,
                      "Calibrated camera. RMS: %f\n"
                      "cam mat:\n%f %f %f\n%f %f %f\n%f %f %f",
                      rms, cam_matrix.at<double>(0, 0),
                      cam_matrix.at<double>(0, 1), cam_matrix.at<double>(0, 2),
                      cam_matrix.at<double>(1, 0), cam_matrix.at<double>(1, 1),
                      cam_matrix.at<double>(1, 2), cam_matrix.at<double>(2, 0),
                      cam_matrix.at<double>(2, 1), cam_matrix.at<double>(2, 2));
        }

        // reset all request vars
        calibration_state->snap_requested = false;
        calibration_state->dump_requested = false;
        calibration_state->matgen_requested = false;
      }

      // Send the pixel data to the display
      cv::imshow("Stream Display", mat);
      cv::waitKey(5);
    }
  }

  ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "Display thread shutting down...");
  return nullptr;
}
#endif

eARCONTROLLER_ERROR OnDecoderConfig(ARCONTROLLER_Stream_Codec_t codec,
                                    void* customData) {
  ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG,
              "Decoder configuration received (codec: %d)", codec.type);

#if DISPLAY_LOCAL_DECODING
  DeviceState_t* device_state = reinterpret_cast<DeviceState_t*>(customData);
  if (codec.type == ARCONTROLLER_STREAM_CODEC_TYPE_H264) {
    // SPS and PPS parameters are stored here. They must be sent to the
    // software decoder.
    auto stream_decoder = new StreamDecoder();
    bool ok =
        stream_decoder->Initialize(4, codec.parameters.h264parameters.spsBuffer,
                                   codec.parameters.h264parameters.spsSize,
                                   codec.parameters.h264parameters.ppsBuffer,
                                   codec.parameters.h264parameters.ppsSize);
    if (!ok) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                  "Failed to initialize software decoder.");
      delete stream_decoder;
      return ARCONTROLLER_OK;
    }

    ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "Initialized software decoder.");
    device_state->stream_decoder = stream_decoder;

#if DISPLAY_OPENCV
    int ret = ARSAL_Thread_Create(&device_state->display_thread, DisplayThread,
                                  (void*)device_state);
    if (ret == 0) {
      ARSAL_PRINT(ARSAL_PRINT_DEBUG, TAG, "Started display thread.");
    } else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                  "Could not start display thread (errno %d %s).", errno,
                  strerror(errno));
    }
#endif
  }
#endif

  return ARCONTROLLER_OK;
}

eARCONTROLLER_ERROR OnReceiveFrame(ARCONTROLLER_Frame_t* frame,
                                   void* customData) {
  DeviceState_t* device_state = reinterpret_cast<DeviceState_t*>(customData);

#if DISPLAY_LOCAL_DECODING
  if (device_state->stream_decoder) {
    auto stream_decoder = device_state->stream_decoder;

    // TODO(justin): Should we be doing decoding in this callback?
    // If decoding takes a long time, frames will be dropped!
    // Problem: ARSDK does not guarantee frame->data will be preserved upon
    // returning from this callback! We'd have to copy the data every frame.
    int ret = stream_decoder->ParseData(frame->data, frame->used);
    if (ret < 0) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Decoder failed to decode a frame!");
    }
  }
#endif

  return ARCONTROLLER_OK;
}

void OnFrameTimeout(void* customData) {}

void OnStateChanged(eARCONTROLLER_DEVICE_STATE newState,
                    eARCONTROLLER_ERROR error, void* customData) {
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Controller state changed to %d",
              newState);

  DeviceState_t* device_state = reinterpret_cast<DeviceState_t*>(customData);
  ARSAL_Sem_Post(&device_state->state_sem);
}

void OnCommandReceived(eARCONTROLLER_DICTIONARY_KEY commandKey,
                       ARCONTROLLER_DICTIONARY_ELEMENT_t* elementDictionary,
                       void* customData) {
  DeviceState_t* device_state = reinterpret_cast<DeviceState_t*>(customData);

  // if the command received is a battery state changed
  if (commandKey ==
      ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* singleElement = NULL;

    if (elementDictionary != NULL) {
      // get the command received in the device controller
      HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                    singleElement);

      if (singleElement != NULL) {
        // get the value
        HASH_FIND_STR(
            singleElement->arguments,
            ARCONTROLLER_DICTIONARY_KEY_COMMON_COMMONSTATE_BATTERYSTATECHANGED_PERCENT,
            arg);

        if (arg != NULL) {
          device_state->battery = arg->value.U8;
          ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Battery level: %d",
                      device_state->battery);
        } else {
          ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "battery arg is NULL");
        }
      } else {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "battery singleElement is NULL");
      }
    } else {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "battery elements is NULL");
    }
  }

  if ((commandKey ==
       ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ALTITUDECHANGED) &&
      (elementDictionary != NULL)) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                  element);
    if (element != NULL) {
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ALTITUDECHANGED_ALTITUDE,
          arg);
      if (arg != NULL) {
        device_state->alt_hiprecision = arg->value.Double;
      }
    }
  }

  if ((commandKey ==
       ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED) &&
      (elementDictionary != NULL)) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                  element);
    if (element != NULL) {
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_ROLL,
          arg);
      if (arg != NULL) {
        device_state->roll = arg->value.Float;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_PITCH,
          arg);
      if (arg != NULL) {
        device_state->pitch = arg->value.Float;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_ATTITUDECHANGED_YAW,
          arg);
      if (arg != NULL) {
        device_state->yaw = arg->value.Float;
      }
    }
  }

  if ((commandKey ==
       ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED) &&
      (elementDictionary != NULL)) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                  element);
    if (element != NULL) {
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED_LATITUDE,
          arg);
      if (arg != NULL) {
        device_state->latitude = arg->value.Double;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED_LONGITUDE,
          arg);
      if (arg != NULL) {
        device_state->longitude = arg->value.Double;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_POSITIONCHANGED_ALTITUDE,
          arg);
      if (arg != NULL) {
        device_state->altitude = arg->value.Double;
      }
    }
  }

  if ((commandKey ==
       ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATION) &&
      (elementDictionary != NULL)) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                  element);
    if (element != NULL) {
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATION_TILT,
          arg);
      if (arg != NULL) {
        device_state->camera_tilt = arg->value.I8;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_ORIENTATION_PAN,
          arg);
      if (arg != NULL) {
        device_state->camera_pan = arg->value.I8;
      }
    }
  }

  if ((commandKey ==
       ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED) &&
      (elementDictionary != NULL)) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                  element);
    if (element != NULL) {
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_FOV,
          arg);
      if (arg != NULL) {
        device_state->camera_fov = arg->value.Float;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_PANMAX,
          arg);
      if (arg != NULL) {
        device_state->camera_pan_max = arg->value.Float;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_PANMIN,
          arg);
      if (arg != NULL) {
        device_state->camera_pan_min = arg->value.Float;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_TILTMAX,
          arg);
      if (arg != NULL) {
        device_state->camera_tilt_max = arg->value.Float;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_COMMON_CAMERASETTINGSSTATE_CAMERASETTINGSCHANGED_TILTMIN,
          arg);
      if (arg != NULL) {
        device_state->camera_tilt_min = arg->value.Float;
      }

      ARSAL_PRINT(
          ARSAL_PRINT_INFO, TAG,
          "Camera Parameters Received (FOV: %f Pan: %f - %f Tilt: %f - %f)",
          device_state->camera_fov, device_state->camera_pan_min,
          device_state->camera_pan_max, device_state->camera_tilt_min,
          device_state->camera_tilt_max);
    }
  }
}

bool exit_requested = false;
#ifndef _WIN32
static void SignalHandler(int signal) {
  if (signal == SIGINT) {
    if (exit_requested) {
      // Exit already requested, just quit.
      exit(1);
    }

    exit_requested = true;
  }
}
#endif

int ReadArgument(char* buf, size_t len) {
  char* s = buf;
  char c = getchar();
  while (c != EOF && c != '\n' && s < buf + len - 2) {
    if (c == ' ') {
      if (s == buf) {
        // skip beginning whitespace
        c = getchar();
        continue;
      } else {
        // end of argument
        break;
      }
    }

    *s++ = c;
    c = getchar();
  }

  *s = '\0';
  return 0;
}

void* InputThread(void* param) {
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Input thread starting...");

  DeviceState_t* device_state = reinterpret_cast<DeviceState_t*>(param);
  CalibrationState_t* calibration_state = device_state->calibration_state;
  while (true) {
    char c = getchar();
    if (c == '\027') {
      // Escape character.
      getchar();

      char k = getchar();
      switch (k) {
        case 'A':  // arrow up
          break;
        case 'B':  // arrow down
          break;
        case 'C':  // arrow right
          break;
        case 'D':  // arrow left
          break;
        default:
          break;
      }

      continue;
    }

    switch (c) {
      case 'c':
        // swap b/t command/calibration
        if (device_state->control_state == CONTROLSTATE_COMMANDING) {
          device_state->control_state = CONTROLSTATE_CALIBRATING;
        } else if (device_state->control_state == CONTROLSTATE_CALIBRATING) {
          device_state->control_state = CONTROLSTATE_COMMANDING;
        }
        break;
      default:
        break;
    }

    auto controller = device_state->controller;
    switch (c) {
      case 'm': {
        // camera
        c = getchar();
        if (c == EOF) {
          ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                      "Camera commands need 2 parameters");
          break;
        }

        switch (c) {
          case 't': {  // tilt (mt<ang>)
            char ang[10];
            int i = ReadArgument(ang, sizeof(ang));
            if (i == 0) {
              ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "usage: mt <angle>");
              break;
            }

            // TODO(justin): Is this threadsafe? The docs say nothing.
            int tilt = atoi(ang);
            ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Tilting camera to %s (%d)", ang,
                        tilt);
            controller->aRDrone3->setCameraOrientationV2Tilt(
                controller->aRDrone3, (float)tilt);
          } break;
          case 'p': {  // pan (mp<ang>)
            char ang[10];
            int i = ReadArgument(ang, sizeof(ang));
            if (i == 0) {
              ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "usage: mp <angle>");
              break;
            }

            // TODO(justin): Is this threadsafe? The docs say nothing.
            int pan = atoi(ang);
            ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Panning camera to %s (%d)", ang,
                        pan);
            controller->aRDrone3->setCameraOrientationV2Pan(
                controller->aRDrone3, (float)pan);
          } break;
        }

        break;
      }
    }

    if (device_state->control_state == CONTROLSTATE_COMMANDING) {
      switch (c) {
        case 'm': {
        }
      }
    } else if (device_state->control_state == CONTROLSTATE_CALIBRATING) {
      // calibration commands
      switch (c) {
        case 'd':
          // dump
          calibration_state->dump_requested = true;
          break;
        case 'p':
          // snap picture
          calibration_state->snap_requested = true;
          break;
        case 'l':
          // matrix
          calibration_state->matgen_requested = true;
          break;
      }
    }
  }
}

int main(int argc, char* argv[]) {
  int status = 0;
  DeviceState_t device_state;
  std::memset(&device_state, 0, sizeof(device_state));
  ARSAL_Print_SetMinimumLevel(ARSAL_PRINT_VERBOSE);

  device_state.calibration_state = new CalibrationState_t;
  std::memset(device_state.calibration_state, 0, sizeof(CalibrationState_t));

#ifndef _WIN32
  /* Set signal handlers */
  struct sigaction sig_action = {
      /* sa_handler */ SignalHandler,
  };

  int ret = sigaction(SIGINT, &sig_action, NULL);
  if (ret < 0) {
    ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Unable to set SIGINT handler : %d(%s)",
                errno, strerror(errno));
    return 1;
  }
#endif

  // Threading setup
  if (status == 0) {
    status = !status ? ARSAL_Sem_Init(&device_state.state_sem, 0, 0) : status;
    if (status != 0) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                  "Failed to initialize semaphores, errno: %d", errno);
    }
  }

  // Step 1: Discovery of the device
  ARDISCOVERY_Device_t* discovery = nullptr;
  if (status == 0) {
    eARDISCOVERY_ERROR error;
    discovery = ARDISCOVERY_Device_New(&error);

    if (error == ARDISCOVERY_OK) {
      error = ARDISCOVERY_Device_InitWifi(
          discovery, ARDISCOVERY_PRODUCT_BEBOP_2, "bebop2", BEBOP_IP_ADDRESS,
          BEBOP_DISCOVERY_PORT);
    }

    if (error != ARDISCOVERY_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARDiscovery Error %s",
                  ARDISCOVERY_Error_ToString(error));
      return 1;
    }
  }

  // Step 2: Create device controller
  eARCONTROLLER_ERROR controller_error;
  if (status == 0) {
    device_state.controller =
        ARCONTROLLER_Device_New(discovery, &controller_error);

    if (controller_error != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARController Error %s",
                  ARCONTROLLER_Error_ToString(controller_error));
      status = 1;
    }
  }

  if (discovery) {
    ARDISCOVERY_Device_Delete(&discovery);
    discovery = nullptr;
  }

  // Step 3: Setup callbacks
  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Registering callbacks...");
  if (status == 0) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "State changed callback");

    controller_error = ARCONTROLLER_Device_AddStateChangedCallback(
        device_state.controller, OnStateChanged, &device_state);
    if (controller_error != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARController Error %s",
                  ARCONTROLLER_Error_ToString(controller_error));
      status = 1;
    }
  }

  if (status == 0) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Command received callback");

    controller_error = ARCONTROLLER_Device_AddCommandReceivedCallback(
        device_state.controller, OnCommandReceived, &device_state);
    if (controller_error != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARController Error %s",
                  ARCONTROLLER_Error_ToString(controller_error));
      status = 1;
    }
  }

  if (status == 0) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Video callbacks");

    controller_error = ARCONTROLLER_Device_SetVideoStreamCallbacks(
        device_state.controller, OnDecoderConfig, OnReceiveFrame,
        OnFrameTimeout, &device_state);
    if (controller_error != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARController Error %s",
                  ARCONTROLLER_Error_ToString(controller_error));
      status = 1;
    }
  }

  // Step 4: Start the device
  if (status == 0) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Starting the device...");

    controller_error = ARCONTROLLER_Device_Start(device_state.controller);
    if (controller_error != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARController Error %s",
                  ARCONTROLLER_Error_ToString(controller_error));
      status = 1;
    }
  }

  if (status == 0) {
    // Wait until the device changes to a running state.
    eARCONTROLLER_DEVICE_STATE state;
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Waiting for device to be ready...");

    do {
      ARSAL_Sem_Wait(&device_state.state_sem);

      state = ARCONTROLLER_Device_GetState(device_state.controller,
                                           &controller_error);
      if (controller_error != ARCONTROLLER_OK) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARController Error %s",
                    ARCONTROLLER_Error_ToString(controller_error));
        status = 1;
        break;
      }
    } while (state != ARCONTROLLER_DEVICE_STATE_RUNNING &&
             state != ARCONTROLLER_DEVICE_STATE_STOPPED);

    if (controller_error != ARCONTROLLER_OK ||
        state != ARCONTROLLER_DEVICE_STATE_RUNNING) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG,
                  "Device did not swap to the running state properly.");
      status = 1;
    }
  }

  if (status == 0) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Device is now in the running state.");
    device_state.control_state = CONTROLSTATE_COMMANDING;

    status = ARSAL_Thread_Create(&device_state.input_thread, InputThread,
                                 reinterpret_cast<void*>(&device_state));
    if (status != 0) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "Failed to create input thread.");
    }
  }

  if (status == 0) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Enable media streaming.");
    controller_error =
        device_state.controller->aRDrone3->sendMediaStreamingVideoEnable(
            device_state.controller->aRDrone3, 1);

    if (controller_error != ARCONTROLLER_OK) {
      ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARController Error %s",
                  ARCONTROLLER_Error_ToString(controller_error));
      status = 1;
    }
  }

  // Main loop
  if (status == 0) {
    ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Entering main loop.");

    eARCONTROLLER_DEVICE_STATE state;
    timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 5 * 1000;  // 5ms wait
    do {
      ARSAL_Sem_Timedwait(&device_state.state_sem, &timeout);

      state = ARCONTROLLER_Device_GetState(device_state.controller,
                                           &controller_error);
      if (controller_error != ARCONTROLLER_OK) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARController Error %s",
                    ARCONTROLLER_Error_ToString(controller_error));
        status = 1;
        break;
      }
    } while (state != ARCONTROLLER_DEVICE_STATE_STOPPED && !exit_requested);
  }

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Shutting down.");

  // Shutdown
  if (device_state.controller) {
    eARCONTROLLER_DEVICE_STATE state;
    state = ARCONTROLLER_Device_GetState(device_state.controller,
                                         &controller_error);
    if (controller_error == ARCONTROLLER_OK &&
        state != ARCONTROLLER_DEVICE_STATE_STOPPED) {
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Stopping device...");
      controller_error = ARCONTROLLER_Device_Stop(device_state.controller);

      if (controller_error != ARCONTROLLER_OK) {
        ARSAL_PRINT(ARSAL_PRINT_ERROR, TAG, "ARController Error %s",
                    ARCONTROLLER_Error_ToString(controller_error));
        status = 1;
      }

      // Controller should now be in a stopping state. Loop until it's
      // completely stopped.
      do {
        ARSAL_Sem_Wait(&device_state.state_sem);
        state = ARCONTROLLER_Device_GetState(device_state.controller,
                                             &controller_error);
      } while (controller_error == ARCONTROLLER_OK &&
               state != ARCONTROLLER_DEVICE_STATE_STOPPED);
      ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Device stopped. %d", state);
    }

    // Now delete the device.
    ARCONTROLLER_Device_Delete(&device_state.controller);
  }

  // Cleanup.
  if (device_state.input_thread) {
    ARSAL_Thread_Join(device_state.input_thread, nullptr);
    ARSAL_Thread_Destroy(&device_state.input_thread);
  }

#if DISPLAY_OPENCV
  if (device_state.stream_decoder) {
    device_state.stream_decoder->Shutdown();
    if (device_state.display_thread) {
      ARSAL_Thread_Join(device_state.display_thread, nullptr);
      ARSAL_Thread_Destroy(&device_state.display_thread);
    }

    delete device_state.stream_decoder;
  }
#endif

  ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Goodbye!");
}
