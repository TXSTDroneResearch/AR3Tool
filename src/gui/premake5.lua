project_root = "../.."
include(project_root.."/tools/build")
local qt = premake.extensions.qt

group("src")
project("AR3Tool-gui")
  uuid("8CB186D0-0109-4FA7-B730-C5948DF3521B")
  kind("StaticLib")
  language("C++")
  qt.enable()

  links({
  })
  defines({
  })
  includedirs({
    project_root.."/third_party/gflags/src",
  })
  files({"*.h", "*.cc"})
