# Foxeer Micro Predator 5 Racing Camera M8 lens


https://www.foxeer.com/foxeer-micro-predator-5-racing-camera-m8-lens-g-304

This has provided the best performance based on onboard recording


# Logs of cameras

## Foxxer  with new ADC

### Checks on the OpenCV
python3 open_cv_check.py 
Backend           : V4L2
Device            : /dev/video0
Negotiated size   : 640x480
Reported FPS      : 30.00
Capture FOURCC    : 'YUYV' (1448695129)
Frame shape       : (480, 640, 3)
Frame dtype       : uint8
OpenCV frame fmt  : BGR


ffmpeg -f v4l2 -list_formats all -i /dev/video0
ffmpeg version 4.2.7-0ubuntu0.1 Copyright (c) 2000-2022 the FFmpeg developers
  built with gcc 9 (Ubuntu 9.4.0-1ubuntu1~20.04.1)
  configuration: --prefix=/usr --extra-version=0ubuntu0.1 --toolchain=hardened --libdir=/usr/lib/aarch64-linux-gnu --incdir=/usr/include/aarch64-linux-gnu --arch=arm64 --enable-gpl --disable-stripping --enable-avresample --disable-filter=resample --enable-avisynth --enable-gnutls --enable-ladspa --enable-libaom --enable-libass --enable-libbluray --enable-libbs2b --enable-libcaca --enable-libcdio --enable-libcodec2 --enable-libflite --enable-libfontconfig --enable-libfreetype --enable-libfribidi --enable-libgme --enable-libgsm --enable-libjack --enable-libmp3lame --enable-libmysofa --enable-libopenjpeg --enable-libopenmpt --enable-libopus --enable-libpulse --enable-librsvg --enable-librubberband --enable-libshine --enable-libsnappy --enable-libsoxr --enable-libspeex --enable-libssh --enable-libtheora --enable-libtwolame --enable-libvidstab --enable-libvorbis --enable-libvpx --enable-libwavpack --enable-libwebp --enable-libx265 --enable-libxml2 --enable-libxvid --enable-libzmq --enable-libzvbi --enable-lv2 --enable-omx --enable-openal --enable-opencl --enable-opengl --enable-sdl2 --enable-libdc1394 --enable-libdrm --enable-libiec61883 --enable-chromaprint --enable-frei0r --enable-libx264 --enable-shared
  libavutil      56. 31.100 / 56. 31.100
  libavcodec     58. 54.100 / 58. 54.100
  libavformat    58. 29.100 / 58. 29.100
  libavdevice    58.  8.100 / 58.  8.100
  libavfilter     7. 57.100 /  7. 57.100
  libavresample   4.  0.  0 /  4.  0.  0
  libswscale      5.  5.100 /  5.  5.100
  libswresample   3.  5.100 /  3.  5.100
  libpostproc    55.  5.100 / 55.  5.100
[video4linux2,v4l2 @ 0xaaaad2e26ee0] Raw       :     yuyv422 :           YUYV 4:2:2 : 720x576 720x480 640x480 480x320 320x240
[video4linux2,v4l2 @ 0xaaaad2e26ee0] Compressed:       mjpeg :          Motion-JPEG : 1920x1080 1280x720 800x600 720x576 720x480 640x480 480x320 320x240





## Cadex with new ADC

### Checks on the OpenCV
python3 open_cv_check.py 
Backend           : V4L2
Device            : /dev/video0
Negotiated size   : 640x480
Reported FPS      : 30.00
Capture FOURCC    : 'YUYV' (1448695129)
Frame shape       : (480, 640, 3)
Frame dtype       : uint8
OpenCV frame fmt  : BGR




ffmpeg -f v4l2 -list_formats all -i /dev/video0
ffmpeg version 4.2.7-0ubuntu0.1 Copyright (c) 2000-2022 the FFmpeg developers
  built with gcc 9 (Ubuntu 9.4.0-1ubuntu1~20.04.1)
  configuration: --prefix=/usr --extra-version=0ubuntu0.1 --toolchain=hardened --libdir=/usr/lib/aarch64-linux-gnu --incdir=/usr/include/aarch64-linux-gnu --arch=arm64 --enable-gpl --disable-stripping --enable-avresample --disable-filter=resample --enable-avisynth --enable-gnutls --enable-ladspa --enable-libaom --enable-libass --enable-libbluray --enable-libbs2b --enable-libcaca --enable-libcdio --enable-libcodec2 --enable-libflite --enable-libfontconfig --enable-libfreetype --enable-libfribidi --enable-libgme --enable-libgsm --enable-libjack --enable-libmp3lame --enable-libmysofa --enable-libopenjpeg --enable-libopenmpt --enable-libopus --enable-libpulse --enable-librsvg --enable-librubberband --enable-libshine --enable-libsnappy --enable-libsoxr --enable-libspeex --enable-libssh --enable-libtheora --enable-libtwolame --enable-libvidstab --enable-libvorbis --enable-libvpx --enable-libwavpack --enable-libwebp --enable-libx265 --enable-libxml2 --enable-libxvid --enable-libzmq --enable-libzvbi --enable-lv2 --enable-omx --enable-openal --enable-opencl --enable-opengl --enable-sdl2 --enable-libdc1394 --enable-libdrm --enable-libiec61883 --enable-chromaprint --enable-frei0r --enable-libx264 --enable-shared
  libavutil      56. 31.100 / 56. 31.100
  libavcodec     58. 54.100 / 58. 54.100
  libavformat    58. 29.100 / 58. 29.100
  libavdevice    58.  8.100 / 58.  8.100
  libavfilter     7. 57.100 /  7. 57.100
  libavresample   4.  0.  0 /  4.  0.  0
  libswscale      5.  5.100 /  5.  5.100
  libswresample   3.  5.100 /  3.  5.100
  libpostproc    55.  5.100 / 55.  5.100
[video4linux2,v4l2 @ 0xaaaae3c3cee0] Raw       :     yuyv422 :           YUYV 4:2:2 : 720x576 720x480 640x480 480x320 320x240
[video4linux2,v4l2 @ 0xaaaae3c3cee0] Compressed:       mjpeg :          Motion-JPEG : 1920x1080 1280x720 800x600 720x576 720x480 640x480 480x320 320x240