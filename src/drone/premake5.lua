project_root = "../.."
include(project_root.."/tools/build")

group("src")
project("AR3Tool-drone")
  uuid("8DE32611-20D8-481D-B901-6103336DAAC9")
  kind("StaticLib")
  language("C++")
  links({
  })
  defines({
  })
  includedirs({
    project_root.."/third_party/gflags/src",
    
    project_root.."/third_party/ARLibs/include",
    project_root.."/third_party/ffmpeg/include",
  })
  files({"*.h", "*.cc"})
