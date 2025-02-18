# Gimbal compatible QGC build

This guide will show you how to build a QGC version that is compatible with the SIYI gimbal.

## Prerequisites

- Git
- Qt 5.15.2
- Qt Creator
- Qt development tools

You can refer to the [Qt Installation Guide](../Qt/README.md) for installing Qt.

## Clone the QGC repository

```bash
git clone git@github.com:DroneLeaf/QGC-SiYi.git
```

## Build QGC using Qt Creator

1. Open the QGC project in Qt Creator.
2. From Project menue, select Manage Kits and make sure that the kit you are using is compatible with the Qt version 5.15.2.
3. Build the project.

## Build QGC using the terminal (Creator failed first time)

```bash
cd QGC-SiYi
mkdir build
cd build
qmake ../qgroundcontrol.pro
make
```
after the build is done, you can run the QGC by running the following command:
```bash
./qgroundcontrol
```

Note: after building from terminal, you can open the project in Qt Creator and build it from there.