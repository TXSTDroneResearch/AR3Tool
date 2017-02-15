-- Most of this code ripped directly from Xenia (github.com/benvanik/xenia)
include("tools/build")
require("third_party/premake-export-compile-commands/export-compile-commands")
require("third_party/premake-qt/qt")

if os.isdir("C:\\Qt") then
  qtpath "C:\\Qt"
end

location(build_root)
targetdir(build_bin)
objdir(build_obj)

includedirs({
  ".",
  "src",
  "third_party",
})

defines({
  "_UNICODE",
  "UNICODE",

  -- TODO(benvanik): find a better place for this stuff.
  "GLEW_NO_GLU=1",
})

vectorextensions("AVX")
flags({
  --"ExtraWarnings",        -- Sets the compiler's maximum warning level.
  "FatalWarnings",        -- Treat warnings as errors.
})
characterset "Unicode"

filter("kind:StaticLib")
  defines({
    "_LIB",
  })

filter("configurations:Checked")
  runtime("Debug")
  defines({
    "DEBUG",
  })
  runtime("Debug")
filter({"configurations:Checked", "platforms:Windows"})
  buildoptions({
    "/RTCsu",   -- Full Run-Time Checks.
  })

filter("configurations:Debug")
  runtime("Debug")
  defines({
    "DEBUG",
    "_NO_DEBUG_HEAP=1",
  })
  runtime("Release")
filter({"configurations:Debug", "platforms:Windows"})
  linkoptions({
    "/NODEFAULTLIB:MSVCRTD",
  })

filter("configurations:Release")
  runtime("Release")
  defines({
    "NDEBUG",
    "_NO_DEBUG_HEAP=1",
  })
  optimize("On")
  flags({
    "LinkTimeOptimization",
  })
  runtime("Release")
filter({"configurations:Release", "platforms:Windows"})
  linkoptions({
    "/NODEFAULTLIB:MSVCRTD",
  })

filter("platforms:Linux")
  system("linux")
  toolset("clang")

filter({"platforms:Linux", "language:C++"})
  buildoptions({
    "-stdlib=libstdc++",
    "-std=c++14",
  })
  links({
    "c++",
  })

filter("platforms:Windows")
  system("windows")
  toolset("msc")
  buildoptions({
    "/MP",      -- Multiprocessor compilation.
    "/wd4100",  -- Unreferenced parameters are ok.
    "/wd4201",  -- Nameless struct/unions are ok.
    "/wd4512",  -- 'assignment operator was implicitly defined as deleted'.
    "/wd4127",  -- 'conditional expression is constant'.
    "/wd4324",  -- 'structure was padded due to alignment specifier'.
    "/wd4189",  -- 'local variable is initialized but not referenced'.
  })
  flags({
    "NoMinimalRebuild", -- Required for /MP above.
  })
  symbols "On"
  defines({
    "_CRT_NONSTDC_NO_DEPRECATE",
    "_CRT_SECURE_NO_WARNINGS",
    "WIN32",
    "_WIN64=1",
    "_AMD64=1",
  })
  linkoptions({
    "/ignore:4006",  -- Ignores complaints about empty obj files.
    "/ignore:4221",
  })
  links({
    "ntdll",
    "wsock32",
    "ws2_32",
    "glu32",
    "opengl32",
    "comctl32",
    "shlwapi",
  })

solution("AR3Tool")
  uuid("4047A9F0-426E-435E-AB07-E32F47548DE8")
  architecture("x86_64")
  if os.is("linux") then
    platforms({"Linux"})
  elseif os.is("windows") then
    platforms({"Windows"})
  end
  configurations({"Checked", "Debug", "Release"})

  include("src/app")
  include("src/base")
  include("src/drone")
  include("src/gui")