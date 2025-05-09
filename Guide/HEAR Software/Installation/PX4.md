# 🚀 Building PX4 for v6X from the forked [DroneLeaf PX4-Autopilot repository](https://github.com/DroneLeaf/PX4-Autopilot)
This repository is a fork of the PX4-Autopilot project, based on commit `1dacb4cdef2d7145754fc788fa8dc482eed74b40`, which corresponds to the `v1.14.3` release. The fork serves as a stable foundation for custom modifications and experimental features tailored to DroneLeaf's specific UAV platform and control needs.

## 📥 Clone the Repository
Use git to clone the repository:

```bash
git clone https://github.com/DroneLeaf/PX4-Autopilot.git
cd PX4-Autopilot
```

## ✅ Toolchain Prerequisites
Ensure your system has the required tools and dependencies for PX4 development.

On Ubuntu, you can install all dependencies using PX4's setup script:

```bash
bash ./Tools/setup/ubuntu.sh
```
💡 Tip: Restart your terminal or source your shell profile after running the script to ensure environment variables are applied.

## 🌿 Checkout the leaf-dev Branch
Switch to the custom development branch DroneLeaf has been using:

```bash
git checkout leaf-dev
```
📝 **Note**: You can check the git history of `leaf-dev` branch to get an overview of all changes applied starting of the `v1.14.3` tag

## ⚙️ Build for FMUv6X
Compile the firmware for the v6X flight controller:

```bash
make px4_fmu-v6x_default
```

📦 This will generate the firmware binary in the `build/px4_fmu-v6x_default/` directory. Note that we flash the firmware with `.px4` binary.

📝 **Note**: We flash the firmware with the `.px4` binary output file.
📝 **Note**: We have been testing this successfully on [Holybro V6X flight controllers](https://holybro.com/products/pixhawk-6x?variant=44390763692221)


---

© 2025 DroneLeaf AI Solutions LLC. All rights reserved.
