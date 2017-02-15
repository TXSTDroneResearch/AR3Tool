project_root = "../.."
include(project_root.."/tools/build")

group("src")
project("AR3Tool-app")
  uuid("5E2D1D58-7717-431F-BB83-C789F3BFBAB8")
  kind("ConsoleApp")
  language("C++")

  filter("platforms:Windows")
    libdirs {
      project_root.."/third_party/ARLibs/mingw64/bin",
      project_root.."/third_party/ffmpeg/msc140/bin"
    }
  filter {}

  libdirs {
    project_root.."/third_party/opencv/build/lib/Release",
  }

  filter("platforms:Windows")
    links({
      "libarcontroller",
      "libardiscovery",
      "libarsal",
    })
  filter("platforms:Linux")
    links({
      "arcontroller",
      "ardiscovery",
      "arsal",
    })
  filter {}

  links({
    --[[
    -- For static libar
    "gcc",
    "arcontroller",
    "ardiscovery",
    "arsal",
    "arcommands",
    "arnetwork",
    "arnetworkal",
    "arstream",
    "arstream2",
    "json",
    --]]

    -- OpenCV
    "opencv_core320",
    "opencv_calib3d320",
    "opencv_features2d320",
    "opencv_flann320",
    "opencv_highgui320",
    "opencv_imgcodecs320",
    "opencv_imgproc320",
    "opencv_ml320",
    "opencv_objdetect320",
    "opencv_photo320",
    "opencv_shape320",
    "opencv_stitching320",
    "opencv_superres320",
    "opencv_video320",
    "opencv_videoio320",
    "opencv_videostab320",

    -- FFmpeg
    "avcodec",
    "avdevice",
    "avfilter",
    "avformat",
    "avutil",
    "swscale",

    "AR3Tool-base",
    "AR3Tool-drone",
    -- "AR3Tool-gui",
  })
  defines({
  })
  includedirs({
    project_root.."/third_party/gflags/src",

    project_root.."/third_party/opencv/build",
    project_root.."/third_party/opencv/include",
    project_root.."/third_party/opencv/modules/calib3d/include",
    project_root.."/third_party/opencv/modules/core/include",
    project_root.."/third_party/opencv/modules/features2d/include",
    project_root.."/third_party/opencv/modules/flann/include",
    project_root.."/third_party/opencv/modules/highgui/include",
    project_root.."/third_party/opencv/modules/imgcodecs/include",
    project_root.."/third_party/opencv/modules/imgproc/include",
    project_root.."/third_party/opencv/modules/ml/include",
    project_root.."/third_party/opencv/modules/objdetect/include",
    project_root.."/third_party/opencv/modules/photo/include",
    project_root.."/third_party/opencv/modules/shape/include",
    project_root.."/third_party/opencv/modules/stitching/include",
    project_root.."/third_party/opencv/modules/superres/include",
    project_root.."/third_party/opencv/modules/video/include",
    project_root.."/third_party/opencv/modules/videoio/include",
    project_root.."/third_party/opencv/modules/videostab/include",

    project_root.."/third_party/ARLibs/include",
    project_root.."/third_party/ffmpeg/include",
  })
  files({"*.h", "*.cc"})
