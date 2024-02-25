# Waveshare ESP32-S3-Touch-LCD-4.3 with Squareline Studio and PlatformIO

This is a first test with getting a Waveshare ESP32-S3-LCD-4.3 Touch running with an exported template project from SquareLine Studio.

It is a very simple two-screen UI with buttons going back and forth.

## Get Started

1. Open the project in PlatformIO
2. Connect your board to USB
3. Make sure to press and hold the Boot button on your board, then press the Reset button, and release the Boot button
4. Click Upload
5. Enjoy clicking back and forth on your touch panel :)

## Libraries

The libraries are the ones provided on the [Waveshare Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-4.3): [S3-4.3-libraries.zip](https://files.waveshare.com/wiki/ESP32-S3-Touch-LCD-4.3/S3-4.3-libraries.zip), except for lvgl, which is added as library dependency in `platformio.ini`.

I tried to add the other two libraries as dependencies as well, but Waveshare made some changes:

- ESP32_IO_Expander: The CH422G chip was added. There are [forks](https://github.com/esp-arduino-libs/ESP32_IO_Expander/network) from the original ESP32 repo including the new files, but nothing official yet.
- ESP32_Display_Panel: There are also some changes in the repo, that I did not yet investigate.

## SquareLine Studio

The setting that worked the best for me in SquareLine Studio was Arduino with TFT_eSPI. I used Export > Create Template Project, but only copied the `lib/ui` folder into this project.

## PlatformIO

Check the profile in platformio.ini:

```
[env:esp32s3box]
platform = espressif32
board = esp32s3box
framework = arduino
monitor_speed = 115200
board_upload.flash_size = 8MB
build_flags = 
	-D BOARD_HAS_PSRAM
	-D LV_CONF_INCLUDE_SIMPLE
	-I lib
board_build.arduino.memory_type = qio_opi
board_build.f_flash = 80000000L
board_build.flash_mode = qio
lib_deps = 
	lvgl/lvgl@8.3.8
```

These are the equivalents for settings the [Waveshare Wiki](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-4.3) recommends for the Arduino IDE.

## Source

The `src/main.cpp` is basically a combination of the `ESP32-S3-Touch-LCD-4.3_Code/Arduino/lvgl_Porting` example from the Waveshare wiki, with removed demo and `#include <ui.h>` from the exported SquareLine Studio project.

## Caveats

There are some yellow artifacts when doing animations. Also, I have not yet figured out how to rotate the screen.

Feedback welcome!
