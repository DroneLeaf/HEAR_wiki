## Installation:
```bash
# refresh package index
sudo apt update

# generic kernel flavour – pulls the matching perf binary automatically
sudo apt install linux-tools-generic   # ≈2 MB

sudo apt install linux-tools-$(uname -r)

sudo apt install ros-noetic-roscpp-dbgsym

sudo apt install ubuntu-dbgsym-keyring

```

to avoid sudo (you need to do this after every restart):
```bash
echo 0 | sudo tee /proc/sys/kernel/perf_event_paranoid
sudo setcap cap_sys_admin,cap_sys_ptrace=eip $(command -v perf)
```

Install hotspot:
```bash
sudo apt install hotspot
```

## Usage
Uncomment in root CMakeLists:
```bash
target_compile_options(px4_mav_opti_flight_controller_onboard_mission PRIVATE -O2 -g -fno-omit-frame-pointer)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")           # keep MSVC happy
  add_compile_options(-O2 -g -fno-omit-frame-pointer)   # applies to every target
endif()
```

Once compiled, run the node with:
```bash
perf record -g --call-graph dwarf -- rosrun flight_controller px4_mav_opti_flight_controller_onboard_mission
```

Then visualize:
```bash
hotspot perf.data
```