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

  filter("platforms:Windows")
    links({
      "libarcontroller",
      "libardiscovery",
      "libarsal",
    })

    libdirs({
      project_root.."/third_party/opencv/install/x64/vc14/lib",
      project_root.."/third_party/ARLibs/mingw64/bin",
      project_root.."/third_party/ffmpeg/msc140/bin"
    })
  filter("platforms:Linux")
    links({
      "arcontroller",
      "ardiscovery",
      "arsal",

      -- for gamepads
      "udev",
    })

    libdirs({
      project_root.."/third_party/ARLibs/gcc/lib",
      project_root.."/third_party/opencv/install/lib",
      project_root.."/third_party/ffmpeg/gcc/lib",

      -- Include local libs
      -- "/usr/local/lib",
    })
  filter {}

  libdirs({project_root.."/third_party/opencv/install/x64/vc14/lib"})
  includedirs({project_root.."/third_party/opencv/install/include"})

  local cv_libs = {
    "opencv_core",
    "opencv_calib3d",
    "opencv_features2d",
    "opencv_flann",
    "opencv_highgui",
    "opencv_imgcodecs",
    "opencv_imgproc",
    "opencv_ml",
    "opencv_objdetect",
    "opencv_photo",
    "opencv_shape",
    "opencv_stitching",
    "opencv_superres",
    "opencv_video",
    "opencv_videoio",
    "opencv_videostab",
  }

  if os.get() == "windows" then
    -- append 320 to lib names
    for i, v in ipairs(cv_libs) do
      cv_libs[i] = cv_libs[i] .. "320"
    end
  end

  links(cv_libs)

  links({
    "libgamepad",

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
    "GAMEPAD_STATIC_LIB",
  })
  includedirs({
    project_root.."/third_party/gflags/src",

    project_root.."/third_party/ARLibs/include",
    project_root.."/third_party/Eigen/include",
    project_root.."/third_party/ffmpeg/include",
  })
  files({"*.h", "*.cc"})
