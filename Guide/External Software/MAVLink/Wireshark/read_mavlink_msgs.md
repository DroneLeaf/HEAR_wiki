** Note: ** the Lua script has been attached to the same directory for convinience.

Follow the guide in https://mavlink.io/en/guide/wireshark.html

Once you generate the WLua script, you need to copy it to the following directory for wireshark to read it as a plugin:
sudo cp ~/mavlink/mavlink/WLua/WLua.lua /usr/lib/x86_64-linux-gnu/wireshark/plugins/3.2/epan/

** Note: ** you might need to change wireshark version in the directory