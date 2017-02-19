group("third_party")
project("libgamepad")
  uuid("bb143d61-3fd4-44c2-8b7e-04cc538ba2c7")
  kind("StaticLib")
  language("C")

  defines({
    "GAMEPAD_STATIC_LIB",
  })
  files({
    "libgamepad/gamepad.c",
    "libgamepad/gamepad.h",
  })
  warnings("Off")