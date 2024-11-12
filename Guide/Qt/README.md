# Qt freamwork installation guide

This guide will show you how to install the Qt framework on your system and manage multiple version installation.

## Prerequisites

- Python 3.8 or later

## Install aqtinstall tool

The aqtinstall tool is a Python script that helps you install and manage Qt versions on your system. You can install it using the following command:

```bash
sudo apt-get install libdbus-1-3 libpulse-mainloop-glib0
sudo pip3 install -U pip
sudo pip install aqtinstall
```

## Install Qt

You can install a specific version of Qt using the aqtinstall tool. For example, to install a specif <Qt_VERSION>, you can use the following command:

```bash
sudo aqt install-qt  --outputdir /opt/Qt linux desktop <QT_VERSION> gcc_64 -m all
```
replace <QT_VERSION> with the version you want to install.

You can check the available versions using the following command:

```bash
aqt list-qt linux desktop
```

## Set the default Qt version

You can set the default Qt version by setting up some environment variables. 
You can add the following lines to your ~/.bashrc file:

```bash
export QT_DEFAULT_VERSION=6.6.3 # <QT_VERSION>
export PATH=/opt/Qt/$QT_DEFAULT_VERSION/gcc_64/bin:$PATH
export LD_LIBRARY_PATH=/opt/Qt/$QT_DEFAULT_VERSION/gcc_64/lib:$LD_LIBRARY_PATH
export QT_PLUGIN_PATH=/opt/Qt/$QT_DEFAULT_VERSION/gcc_64/plugins/
export QML_IMPORT_PATH=/opt/Qt/$QT_DEFAULT_VERSION/gcc_64/qml/
export QML2_IMPORT_PATH=/opt/Qt/$QT_DEFAULT_VERSION/gcc_64/qml/
```

## Verify the installation

You can verify the installation by running the following command:

```bash
qmake -v
```
You should see the Qt version you installed.

## Qt Creater installation

You can install the Qt Creator you want that is also cpmatible with your Qt version by downloading the installation .run script from [Qt Archive](https://download.qt.io/archive/) and running it.

```bash
chmod +x qt-creator-opensource-linux-x86_64-4.15.0.run # example
./qt-creator-opensource-linux-x86_64-4.15.0.run
```

### OR

Using the Qt Online Installer, create an account and you can manage the installation of Qt Creator and other tools. (Doesn't show all versions of Qt as aqtinstall)

## Conclusion

In this guide, you learned how to install the Qt framework on your system and manage multiple version installation. You also learned how to set the default Qt version and verify the installation. You can now start developing applications using the Qt framework.