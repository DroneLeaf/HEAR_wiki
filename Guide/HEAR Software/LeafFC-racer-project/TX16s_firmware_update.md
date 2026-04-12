
Get firmware from; [https://github.com/EdgeTX/edgetx/releases](https://github.com/EdgeTX/edgetx/releases)
This is the download link for [v2.12.0](https://github.com/EdgeTX/edgetx/releases/download/v2.12.0/edgetx-firmware-v2.12.0.zip)

Similarly get SD card from [v2.12.0](https://github.com/EdgeTX/edgetx-sdcard/releases/tag/v2.12.0)
for TX16s it is: **c480x272.zip** (480x272 pixel, landscape orientation)

Place the files in the `/Downloads` folder and unzip them

> [!NOTE]  
> Replace the path `/media/droneleaf/44D3-81F9` below with the actual SD card once mounted in your laptop

Copy this firmware file:

`~/Downloads/edgetx-firmware-v2.12.0/tx16s-9249f89.bin`

Copy it into this folder on the radio SD card:

`/media/droneleaf/44D3-81F9/FIRMWARE/`

EdgeTX only shows firmware files from the `FIRMWARE` folder when flashing from the radio.

So the exact command is:

~~~bash
cp ~/Downloads/edgetx-firmware-v2.12.0/tx16s-9249f89.bin /media/droneleaf/44D3-81F9/FIRMWARE/
~~~

About your saved models

Your models are stored in the `MODELS` folder on the SD card in EdgeTX.

So:

- Flashing the firmware file does not by itself delete your models
- But blindly replacing the SD card contents can overwrite your `MODELS` folder, which is where your model files live.

Safest backup first

Run this before changing anything:

~~~bash
mkdir -p ~/tx16s-backup
sudo cp -a /media/droneleaf/44D3-81F9 ~/tx16s-backup/SD-backup-$(date +%F)
~~~

That gives you a full backup of the current SD card.

If you want extra safety, on the radio itself you can also back up a model from the model management screen; EdgeTX documents a `Backup Model` function that writes a copy to the SD card backup area.

What to copy from `c480x272/`

Since you are already on EdgeTX and want a safe update, do this:

Copy the new SD card contents onto the SD card but do not overwrite your current `MODELS` folder unless you have a backup and want to replace it.

A practical safe command is:

~~~bash
sudo rsync -av \
  --exclude MODELS \
  ~/Downloads/c480x272/ /media/droneleaf/F2D5-48A9/
~~~

That updates the standard EdgeTX SD card folders while leaving your existing models alone.

I would also preserve your current `RADIO` folder unless you specifically want to replace radio-level settings:

~~~bash
sudo rsync -av \
  --exclude MODELS \
  --exclude RADIO \
  ~/Downloads/c480x272/ /media/droneleaf/44D3-81F9 /
~~~

Do this on the TX16S:

1. Power the radio on normally.
2. Open the SD card browser.
3. Go into the `FIRMWARE` folder.
4. Highlight `tx16s-9249f89.bin`.
5. Long-press `ENTER` on that file.
6. Choose `Flash bootloader`. EdgeTX says you will only see this option if the file matches your radio target.

Then:

1. Go back and power the radio off.
2. Enter bootloader mode by turning it on while pulling both horizontal trims together. The EdgeTX guide lists TX16S MK1/MK2 under the radios that enter bootloader this way.
3. In the bootloader screen, choose `Write Firmware`.
4. Select `tx16s-9249f89.bin` again.
5. Long-press to flash it.

After that, reboot normally and check:

- `SYS → VERSION`
- it should now show `EdgeTX 2.12.0` instead of `2.7.1`.

A few important notes:

- Copying the SD card pack (`c480x272`) does not itself update the radio firmware; it only updates the SD contents used by EdgeTX.
- Your saved models should remain as long as you did not overwrite or delete your `MODELS` folder on the SD card.
- EdgeTX’s joystick docs say the default Classic HID report has 8 analog axes and 24 buttons, and Advanced mode lets you configure outputs individually.

If you update to `2.12.0` and still do not see the USB joystick section, then the next thing to check is whether it appears lower down in `Model Setup` or whether your build/target omits it.