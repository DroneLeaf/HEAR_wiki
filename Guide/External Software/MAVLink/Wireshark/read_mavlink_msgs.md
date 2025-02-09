** Note: ** the Lua script has been attached to the same directory for convenience.

Follow the guide in https://mavlink.io/en/guide/wireshark.html

TL;DR: run the following command in the mavlink repo root folder:
```
python3 -m mavgenerate
```
From the GUI select `all.xml` and choose the `WLua` folder as destination, also choose WLua as language with v2.0. Now a `WLua.lua` file would be created in the root repo folder of mavlink.

Once you generate the WLua script, you need to copy it to the following directory for wireshark to read it as a plugin (**Make sure you are in mavlink repo root directory**):
```
sudo cp WLua.lua /usr/lib/x86_64-linux-gnu/wireshark/plugins/[wire_shark_ver]/epan/
```

** Note: ** you might need to change wireshark version in the directory [wire_shark_ver]

** Restart Wireshark for the new plugin to take effect**