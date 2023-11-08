
# Adding Dependencies

To add dependencies please adhere to the following guidelines:

## Adding to CMake

Under `Components` section in `Configurations.cmake` add the dependency as an option. This would allow turning on/off a dependency for a specific build.

  

*Example:*

  
```
option(GUI_Display "GUI Display" ON)

if(GUI_Display)

add_definitions(-DGUI_Display)

endif()
```
Now `GUI_Display` can be checked within all source files by using `#ifdef`. See `GraphDisplayDriverG.hpp` for an example of checking on `GUI_Display`.

  

In the above example, `GUI_Display` is used to show mission pipeline graphs on the screen using GTK library. If you check the code you can find that all GTK usages in `CMakeLists.txt` are guarded by checking if `GUI_Display` option is turned ON, for example:

##### GUI_Display #####
```
if(GUI_Display)

pkg_check_modules(GTKMM REQUIRED gtkmm-3.0)

find_package(OpenCV REQUIRED)

link_directories(

${GTKMM_LIBRARY_DIRS}

${OpenCV_INCLUDE_DIRS}

)

endif()

##### GUI_Display #####
```
  
  

## Documenting

Add installation instructions of the new dependency to the following headline *Installing Dependencies Instructions*. Please follow the same format

  

# Installing Dependencies Instructions (Please move to the most suitable repo)

  

### Installing vcpkg | Owner: Ahmed Hashim | Needed for: WEB

  

```bash

cd

git  clone  https://github.com/microsoft/vcpkg  ./vcpkg/bootstrap-vcpkg.sh

cd  ~/vcpkg

./vcpkg  install  cpr

./vcpkg  install  rapidjson

./vcpkg  install  boost-Context

./vcpkg  install  boost-Filesystem

./vcpkg  install  boost-System

./vcpkg  install  boost-Regex

./vcpkg  install  boost

```

  

### Installing gtkmm-3.0 | Owner: Chehadeh | Needed for: GUI

  

```bash

sudo  apt-get  install  libgtk-3-dev

sudo  apt-get  install  libgtkmm-3.0-dev

sudo  apt-get  install  libgstreamermm-1.0-dev

```

  

### Installing opencv | Owner: Chehadeh | Needed for: GUI

  

```bash

sudo  apt-get  install  graphviz-dev

wget  -O  opencv.zip  https://github.com/opencv/opencv/archive/4.0.0.zip

wget  -O  opencv_contrib.zip  https://github.com/opencv/opencv_contrib/archive/4.0.0.zip

unzip  opencv.zip

unzip  opencv_contrib.zip

cd  ~/opencv-4.0.0

mkdir  build

cd  build

cmake  -D  CMAKE_BUILD_TYPE=RELEASE  \

-D CMAKE_INSTALL_PREFIX=/usr/local \

-D  OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib-4.0.0/modules  \

-D ENABLE_NEON=ON \

-D  ENABLE_VFPV3=ON  \

-D BUILD_TESTS=OFF \

-D  WITH_TBB=OFF  \

-D INSTALL_PYTHON_EXAMPLES=OFF \

-D  BUILD_EXAMPLES=OFF  ..

make  –j4

sudo  apt-get  install  libopencv-devpython-opencv

```

  

### Installing pcap | Owner: Chehadeh | Needed for: pcap driver

  

```bash

sudo  apt-get  install  libpcap-dev

```
