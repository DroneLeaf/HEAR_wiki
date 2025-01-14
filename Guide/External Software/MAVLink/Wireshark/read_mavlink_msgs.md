** Note: ** the Lua script has been attached to the same directory for convenience.

Follow the guide in https://mavlink.io/en/guide/wireshark.html

TL;DR: run the following command in the mavlink repo root folder:
```
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=mavlink_2_common message_definitions/all.xml
```

Once you generate the WLua script, you need to copy it to the following directory for wireshark to read it as a plugin:
```
sudo cp ~/mavlink/mavlink/WLua/WLua.lua /usr/lib/x86_64-linux-gnu/wireshark/plugins/[wire_shark_ver]/epan/
```

** Note: ** you might need to change wireshark version in the directory [wire_shark_ver]