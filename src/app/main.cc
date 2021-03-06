#include <errno.h>
#include <signal.h>
#include <sys/types.h>
#include <chrono>
#include <cstring>

#include <vector>

extern "C" {
#include <libARController/ARController.h>
#include <libARDiscovery/ARDiscovery.h>
#include <libARSAL/ARSAL.h>
}

#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include "drone/region.h"
#include "drone/stream_decoder.h"

#include "third_party/libgamepad/gamepad.h"

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

const ControlState initial_state_ = CONTROLSTATE_COMMANDING;

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
  bool running;

  ARSAL_Thread_t input_thread;

#if DISPLAY_OPENCV
  ARSAL_Thread_t display_thread;
#endif

  ControlState control_state;
  CalibrationState_t* calibration_state;

  uint32_t flying_state;
  uint8_t battery;
  float vel_x, vel_y, vel_z;  // velocity: (+N/S), (+E/W), +down
  float pitch, yaw, roll;     // yaw: [-pi, pi], pitch & roll: [-pi/2, pi/2]
  double latitude, longitude, altitude;
  double alt_hiprecision;
  int gps_status;

  uint8_t camera_stabilization;  // bitfield: pitch | roll
  float camera_fov;
  int8_t camera_pan, camera_tilt;
  float camera_pan_min, camera_pan_max;
  float camera_tilt_min, camera_tilt_max;
  float camera_tilt_maxvel, camera_pan_maxvel;
};

// Converts a given Euler angles to Rotation Matrix
cv::Mat ConvertEulerToMatrixYPR(const cv::Mat& euler) {
  double ch = cos(euler.at<double>(0));
  double sh = sin(euler.at<double>(0));
  double ca = cos(euler.at<double>(1));
  double sa = sin(euler.at<double>(1));
  double cb = cos(euler.at<double>(2));
  double sb = sin(euler.at<double>(2));

  // clang-format off
  // Rotation about z-axis (roll/bank) +clockwise
  cv::Mat Rz = (cv::Mat_<double>(3, 3) <<
    1,  0,   0,
    0, cb, -sb,
    0, sb,  cb
  );

  // Rotation about y-axis (yaw/heading) +clockwise
  cv::Mat Ry = (cv::Mat_<double>(3, 3) <<
     sh, 0, ch,
      0, 1,  0,
    -ch, 0, sh
  );

  // Rotation about x-axis (pitch/attitude) +up/-down
  cv::Mat Rx = (cv::Mat_<double>(3, 3) <<
    ca, -sa, 0,
    sa,  ca, 0,
     0,   0, 1
  );
  // clang-format on

  // Rotation order: YPR
  return (Rz * Rx) * Ry;
}

cv::Point3f operator*(cv::Mat M, const cv::Point3f& p) {
  cv::Mat_<float> src(3, 1);
  src(0, 0) = p.x;
  src(1, 0) = p.y;
  src(2, 0) = p.z;

  cv::Mat_<float> dst = M * src;
  return cv::Point3f(dst(0, 0), dst(1, 0), dst(2, 0));
}

#if DISPLAY_OPENCV
void* DisplayThread(void* param) {
  DeviceState_t* device_state = reinterpret_cast<DeviceState_t*>(param);
  StreamDecoder* stream_decoder = device_state->stream_decoder;
  cv::namedWindow("Stream Display");

  // Test object
  drone::Region test_region;
  test_region.add_point_deg({0.0, 0.0, 0.0});  // House
  // test_region.add_point_deg({0.0, 0.0, 0.0});  // Centennial

  DecodedFrame_t frame;
  std::memset(&frame, 0, sizeof(frame));
  while (true) {
    int ret = stream_decoder->GetFrame(&frame);
    if (ret > 0) {
      break;
    }

    char key = cv::waitKey(1);
    if (ret == 0) {
      auto width = frame.width;
      auto height = frame.height;

      cv::Mat mat(height, width, CV_8UC3, frame.data);
      if (device_state->control_state == CONTROLSTATE_COMMANDING) {
        cv::String stats_loc;
        if (device_state->gps_status != 0) {
          stats_loc =
              cv::format("GPS: %f %f %f", device_state->latitude,
                         device_state->longitude, device_state->altitude);
        } else {
          stats_loc =
              cv::format("GPS: NO DATA", device_state->latitude,
                         device_state->longitude, device_state->altitude);
        }
        auto stats_rot = cv::format("YPR: %f %f %f", device_state->yaw,
                                    device_state->pitch, device_state->roll);
        auto stats_vel = cv::format("VEL: %f %f %f", device_state->vel_x,
                                    device_state->vel_y, device_state->vel_z);
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
        cv::putText(mat, stats_vel, cv::Point(20, 60),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));
        cv::putText(mat, stats_cam, cv::Point(20, 80),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));
        cv::putText(mat, stats_alt, cv::Point(20, 100),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));
        cv::putText(mat, stats_bat, cv::Point(20, 120),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));

        // Calibration matrix: Obtained from taking pictures of a checkerboard
        // in calibration mode.
        // TODO(justin): Unhardcode this!
        // [fx, 0, cx]
        // [0, fy, cy]
        // [0,  0,  1]
        // 439.705726   0.000000 440.135793
        //   0.000000 423.592382 281.677219
        //   0.000000   0.000000   1.000000
        double _cm[9] = {
            439.705726, 0, 440, 0, 423.592382, 280, 0, 0, 1,
        };
        cv::Mat cam_matrix = cv::Mat(3, 3, CV_64FC1, _cm);
        cv::Vec4d dcoeffs(0, 0, 0, 0);

        // Rotation matrix: How much the virtual camera should be rotated to
        // match the orientation of the physical camera
        // Pitch and roll can be stabilized by the drone. If stabilization is
        // enabled, pitch is always stabilized and roll is optional.
        float cam_tilt = -deg2rad((float)device_state->camera_tilt);
        float pitch = device_state->camera_stabilization & 0x2
                          ? cam_tilt
                          : -device_state->pitch + cam_tilt;

        float roll =
            device_state->camera_stabilization & 0x1 ? 0 : -device_state->roll;

        cv::Mat rot_matrix = ConvertEulerToMatrixYPR(cv::Mat(
            cv::Vec3d(-device_state->yaw + (CV_PI * 3.0 / 2.0), pitch, roll)));

        // clang-format off
        cv::Mat Rp = (cv::Mat_<double>(3, 3) <<
           0,  0, 1,
           0, -1, 0,
           1,  0, 0
        );
        // clang-format on

        // -> pinhole camera model
        rot_matrix = Rp * rot_matrix;

        cv::Vec3d rvec;
        cv::Rodrigues(rot_matrix, rvec);

        // Delta: (W/E, N/S, altitude)
        auto delta =
            test_region.GetLinearDelta(0, {deg2rad(device_state->longitude),
                                           deg2rad(device_state->latitude),
                                           device_state->alt_hiprecision});

        auto stats_del =
            cv::format("DEL: %f %f %f", delta.x(), delta.y(), delta.z());

        cv::putText(mat, stats_del, cv::Point(20, 140),
                    cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                    cv::Scalar(255, 255, 255));

        // Point delta: (N/S, altitude, E/W)
        std::vector<cv::Point3f> points3d(1);
        points3d[0] = {(float)delta.y(), (float)delta.z(),
                       (float)delta.x()};  // origin
        // points3d[0] = {1.5f, (float)delta.z(), 0.f};
        points3d.push_back(points3d[0] + cv::Point3f(0.f, 0.f, 0.f));
        points3d.push_back(points3d[0] + cv::Point3f(1.f, 0.f, 0.f));
        points3d.push_back(points3d[0] + cv::Point3f(0.f, 0.f, 1.f));
        points3d.push_back(points3d[0] + cv::Point3f(1.f, 0.f, 1.f));

        // TODO(justin): projectPoints does not do culling. Points behind the
        // camera are still projected onscreen (up-side-down)
        std::vector<cv::Point2f> points2d(points3d.size());
        cv::Vec3d tvec(0, 0, 0);
        cv::projectPoints(points3d, rvec, tvec, cam_matrix, dcoeffs, points2d);

        for (int i = 1; i < points2d.size(); i++) {
#if 0
          auto row = cv::format("%f %f %f", rot_matrix.at<double>(i, 0),
                                rot_matrix.at<double>(i, 1),
                                rot_matrix.at<double>(i, 2));
          cv::putText(mat, row, cv::Point(20, 120 + i * 20),
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                      cv::Scalar(255, 255, 255));
#endif

          cv::Scalar col(0.0);
          col[(i - 1) % 3] = 255;
          cv::circle(mat, points2d[i], 10, col, -1);
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
        // Board size in INSIDE CORNERS of black squares. If you have the wrong
        // size, OpenCV won't see the board. (height, width)
        cv::Size board_size(9, 14);
        std::vector<cv::Vec2f> image_points;
        bool found =
            cv::findChessboardCorners(grayscale, board_size, image_points);
        if (found) {
          cv::putText(mat, "found checkerboard", cv::Point(20, 100),
                      cv::HersheyFonts::FONT_HERSHEY_PLAIN, 1.f,
                      cv::Scalar(0, 255, 0));
          cv::drawChessboardCorners(mat, board_size, image_points, found);

          // if (calibration_state->snap_requested) {
          // Store in array.
          ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Snapshotting checkerboard...");
          calibration_state->image_points.push_back(image_points);
          // }
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

        if (/* calibration_state->matgen_requested && */
            calibration_state->image_points.size() > 16) {
          ARSAL_PRINT(ARSAL_PRINT_INFO, TAG, "Generating matrix...");

          // calibrate camera.
          cv::Size image_size(width, height);

          // build object points (the base, CV will find camera pos from this)
          // Note: If we actually calculate the base position, rvec and tvec
          // won't be useless!
          std::vector<cv::Vec3f> obj_corners;
          float square_size = 0.0179f;  // in metres
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
          ARSAL_PRINT(
              ARSAL_PRINT_INFO, TAG,
              "Calibrated camera. RMS: %f\n"
              "cam mat:\n%f %f %f\n%f %f %f\n%f %f %f\n"
              "Distortion: %f %f %f %f %f %f %f %f",
              rms, cam_matrix.at<double>(0, 0), cam_matrix.at<double>(0, 1),
              cam_matrix.at<double>(0, 2), cam_matrix.at<double>(1, 0),
              cam_matrix.at<double>(1, 1), cam_matrix.at<double>(1, 2),
              cam_matrix.at<double>(2, 0), cam_matrix.at<double>(2, 1),
              cam_matrix.at<double>(2, 2), dist_coeffs.at<double>(0, 0),
              dist_coeffs.at<double>(0, 1), dist_coeffs.at<double>(0, 2),
              dist_coeffs.at<double>(0, 3), dist_coeffs.at<double>(0, 4),
              dist_coeffs.at<double>(0, 5), dist_coeffs.at<double>(0, 6),
              dist_coeffs.at<double>(0, 7));

          calibration_state->image_points.clear();
        }

        // reset all request vars
        calibration_state->snap_requested = false;
        calibration_state->dump_requested = false;
        calibration_state->matgen_requested = false;
      }

      // Send the pixel data to the display
      cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);
      cv::imshow("Stream Display", mat);
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
       ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED) &&
      (elementDictionary != NULL)) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                  element);
    if (element != NULL) {
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDX,
          arg);
      if (arg != NULL) {
        device_state->vel_x = arg->value.Float;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDY,
          arg);
      if (arg != NULL) {
        device_state->vel_y = arg->value.Float;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_SPEEDCHANGED_SPEEDZ,
          arg);
      if (arg != NULL) {
        device_state->vel_z = arg->value.Float;
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
       ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_GPSSETTINGSSTATE_GPSFIXSTATECHANGED) &&
      (elementDictionary != NULL)) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                  element);
    if (element != NULL) {
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_GPSSETTINGSSTATE_GPSFIXSTATECHANGED_FIXED,
          arg);
      if (arg != NULL) {
        device_state->gps_status = arg->value.U8;
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

  if ((commandKey ==
       ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_VIDEOSTABILIZATIONMODECHANGED) &&
      (elementDictionary != NULL)) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                  element);
    if (element != NULL) {
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PICTURESETTINGSSTATE_VIDEOSTABILIZATIONMODECHANGED_MODE,
          arg);
      if (arg != NULL) {
        auto val = arg->value.I32;
        if (val ==
            ARCOMMANDS_ARDRONE3_PICTURESETTINGSSTATE_VIDEOSTABILIZATIONMODECHANGED_MODE_ROLL_PITCH) {
          device_state->camera_stabilization = 0x3;
        } else if (
            val ==
            ARCOMMANDS_ARDRONE3_PICTURESETTINGSSTATE_VIDEOSTABILIZATIONMODECHANGED_MODE_PITCH) {
          device_state->camera_stabilization = 0x2;
        } else if (
            val ==
            ARCOMMANDS_ARDRONE3_PICTURESETTINGSSTATE_VIDEOSTABILIZATIONMODECHANGED_MODE_ROLL) {
          device_state->camera_stabilization = 0x1;
        } else {
          device_state->camera_stabilization = 0x0;
        }
      }
    }
  }

  // Camera Velocity
  if ((commandKey ==
       ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_VELOCITYRANGE) &&
      (elementDictionary != NULL)) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = NULL;
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                  element);
    if (element != NULL) {
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_VELOCITYRANGE_MAX_TILT,
          arg);
      if (arg != NULL) {
        device_state->camera_tilt_maxvel = arg->value.Float;
      }
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_CAMERASTATE_VELOCITYRANGE_MAX_PAN,
          arg);
      if (arg != NULL) {
        device_state->camera_pan_maxvel = arg->value.Float;
      }
    }
  }

  // if the command received is a flying state changed
  if ((commandKey ==
       ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED) &&
      (elementDictionary != NULL)) {
    ARCONTROLLER_DICTIONARY_ARG_t* arg = NULL;
    ARCONTROLLER_DICTIONARY_ELEMENT_t* element = NULL;

    // get the command received in the device controller
    HASH_FIND_STR(elementDictionary, ARCONTROLLER_DICTIONARY_SINGLE_KEY,
                  element);
    if (element != NULL) {
      // get the value
      HASH_FIND_STR(
          element->arguments,
          ARCONTROLLER_DICTIONARY_KEY_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE,
          arg);

      if (arg != NULL) {
        device_state->flying_state = arg->value.I32;
      }
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

#if 0
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
      // No commands yet!
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
#endif

void* GamepadInputThread(void* param) {
  DeviceState_t* device_state = reinterpret_cast<DeviceState_t*>(param);

  GamepadInit();
  eARCONTROLLER_ERROR controller_error;
  eARCONTROLLER_DEVICE_STATE state;

  double dt = 0.01;
  double cam_dt = 0.0;
  auto drone = device_state->controller->aRDrone3;
  while (device_state->running) {
    auto start = std::chrono::high_resolution_clock::now();

    state = ARCONTROLLER_Device_GetState(device_state->controller,
                                         &controller_error);
    if (controller_error != ARCONTROLLER_OK) {
      break;
    }

    char key = cv::waitKey(10);
    GamepadUpdate();
    if (state == ARCONTROLLER_DEVICE_STATE_RUNNING) {
      if (device_state->flying_state ==
          ARCOMMANDS_ARDRONE3_PILOTINGSTATE_FLYINGSTATECHANGED_STATE_LANDED) {
        if (GamepadButtonTriggered(GAMEPAD_0, BUTTON_START)) {
          drone->sendPilotingTakeOff(drone);
        }

        if (key == 't') {
          drone->sendPilotingTakeOff(drone);
        }
      } else {
        if (GamepadButtonTriggered(GAMEPAD_0, BUTTON_BACK)) {
          drone->sendPilotingLanding(drone);
        } else if (GamepadButtonTriggered(GAMEPAD_0, BUTTON_B)) {
          drone->sendPilotingEmergency(drone);
        }

        if (key == 't') {
          drone->sendPilotingLanding(drone);
        } else if (key == 'e') {
          drone->sendPilotingEmergency(drone);
        }

        // triggers
        if (GamepadIsConnected(GAMEPAD_0)) {
          // sticks
          Eigen::Vector2f ls, rs;
          GamepadStickNormXY(GAMEPAD_0, STICK_LEFT, &ls.x(), &ls.y());
          GamepadStickNormXY(GAMEPAD_0, STICK_RIGHT, &rs.x(), &rs.y());

          // Cube the sticks for fine-tune control
          ls = ls.cwiseProduct(ls).cwiseProduct(ls);
          rs = rs.cwiseProduct(rs).cwiseProduct(rs);

          float lt = GamepadTriggerLength(GAMEPAD_0, TRIGGER_LEFT);
          float rt = GamepadTriggerLength(GAMEPAD_0, TRIGGER_RIGHT);

          // Scale up
          lt *= -100;
          rt *= 100;

          int8_t roll = (int8_t)(ls.x() * 100.0);
          int8_t pitch = (int8_t)(ls.y() * 100.0);
          int8_t yaw = (int8_t)(rs.x() * 100.0);
          int8_t gaz = (int8_t)(lt + rt);

          drone->setPilotingPCMD(drone, 1, roll, pitch, yaw, gaz, 0);
        }
      }

// CAUTION: Enabling this code freezes the drone! Yay!
#if 0
      cam_dt += dt;
      if (cam_dt > 0.15) {
        cam_dt = 0.0;
        double tilt_del = rs.y() * 360.0 * 2.0 * dt;
        drone->setCameraOrientationV2Tilt(drone,
                                          device_state->camera_tilt + tilt_del);
      }
#endif
    }

    std::chrono::duration<double> delta =
        std::chrono::high_resolution_clock::now() - start;
    dt = delta.count();
  }

  return nullptr;
}

int main(int argc, char* argv[]) {
  int status = 0;
  DeviceState_t device_state;
  std::memset(&device_state, 0, sizeof(device_state));
  ARSAL_Print_SetMinimumLevel(ARSAL_PRINT_INFO);
  device_state.running = true;

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
    device_state.control_state = initial_state_;

    status = ARSAL_Thread_Create(&device_state.input_thread, GamepadInputThread,
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
  device_state.running = false;
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
