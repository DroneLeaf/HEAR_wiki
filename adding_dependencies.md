
# Managing Dependencies
**Document Scope**: adding external header/static/dynamic library dependencies to the source code.

To add dependencies please adhere to the following guidelines:

## Adding to CMake

Under `Components` section in `Configurations.cmake` add the dependency as an option. This would allow turning on/off a dependency for a specific build.


*Example from `Configurations.cmake`:*

  
```
option(GUI_Display "GUI Display" ON)

if(GUI_Display)

add_definitions(-DGUI_Display)

endif()
```

`GUI_Display` is a cmake variable that is used to turn on/off showing mission pipeline graphs on the screen using GTK library. When `GUI_Display` is set to `OFF` all dependencies on GTK library should be removed which covers include and link dependencies. First, this needs to happen in all CMakeLists, whenever GTK dependencies are needed, for example:

```
##### GUI_Display #####

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

Now `GUI_Display` cmake variable defines a precomplier directive `GUI_Display` (line `add_definitions(-DGUI_Display)`) that can be checked within all source files by using `#ifdef`. See `GraphDisplayDriverG.hpp` for an example of checking on `GUI_Display`.

Please observe the following rule when using #ifdef:
1. Your code must be written such that all direct dependency usage (i.e. including header files of the dependent package) must be in a seperate folder with source files. e.g. `GraphDisplay` folder. We refer to these as *direct sources*.
2. Direct sources must be excluded as follows: header files `hpp` by `#ifdef` checks, and `cpp` files in the `CMakeLists` file.
3. Inculsion of the direct sources headers in other source files must be guarded by `#ifdef` and all usages of member classes and variables must be done accordingly.
4. Lets say I have sources `A.hpp`, `B.hpp`, `C.hpp`, and `A.hpp` is the only direct source. But `B.hpp`, `C.hpp` only exist in a meaningful way when `A.hpp` exists, then `B.hpp`, `C.hpp` are also considered as direct sources and you should use the rule #2 above with `B.hpp`, `C.hpp`.
  
## Structuring `Configurations.cmake`
The file `Configurations.cmake` can grow messy easily. So it has to be structured carefully. 

**Definitions:**

- *Hardware Variant*: this refers to a property of the UAV hardware that exists for every UAV. For example, a UAV must have a propulsion configuration. Further to this example, a UAV can only be EXCLUSIVELY a quadrotor OR a hexarotor, it cannot be without a propuslion.
- *Hardware Components*: this refers to an optional additional property of the UAV. For example, a camera for vision.
- *Operating Modes*: this is TEMPORARY PROVISION. DC-DFBC is an example of such operating mode. However, all operating modes must be controlled, in the future, from the main calling code.
- *Target Systems*: this must be the compute setup that would run HEAR. For example, RPI4_UBUNTU_SERVER_20 means the target is Raspberry PI 4 running Ubuntu Server 20.04.
- *Features*: These are additional OPTIONAL software features that are not necessary for core operation. Examples include HEAR_WEB that enables licensing and settings retreival.
- *Software Dependency:* this is the minimal direct dependency. For example, if we need GTM library that depends on OpenCV, and all Features, Target Systems, Hardware Components, and Hardware Variant never depend on OpenCV directly then GTM is a Software Dependency for HEAR, and OpenCV is not.

**Rules:**

- Use software dependency package name. For example, use "OpenCV" not "Vision". Vision is the feature but not the software dependency. Same appiles to GUI_Display, it is not compliant.
- Define software dependencies based on "Features", and "Target Systems". "Features", and "Target Systems" are internat to `Configurations.cmake`.
- No defined "Features", nor "Target Systems" should appear in any CMakeLists.
- Only "Software Dependency", "Hardware Variant", or "Operating Modes" (TEMPORAL) can appear in CMakeLists or source files.

## Documenting

Add installation instructions of the new dependency to the following headline *Installing Dependencies Instructions*. Please follow the same format

# Current Targets

## RPI_



# Installing Dependencies Instructions (TEMPORARY: Please move to the most suitable repo)

  

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
