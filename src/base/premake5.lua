project_root = "../.."
include(project_root.."/tools/build")

group("src")
project("AR3Tool-base")
  uuid("D61CFAD6-07E9-47BF-80E2-6B773D7814DB")
  kind("StaticLib")
  language("C++")
  links({
  })
  filter("platforms:Windows")
    links({"legacy_stdio_definitions"})
  filter()

  defines({
  })
  includedirs({
    project_root.."/third_party/gflags/src",
  })
  files({"*.h", "*.cc"})
