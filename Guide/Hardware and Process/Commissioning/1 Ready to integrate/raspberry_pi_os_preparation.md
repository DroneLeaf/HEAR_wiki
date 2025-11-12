# Raspberry Pi's OS Deployment (OS only without HEAR stack)

This guide covers preparing Raspberry Pi Model B and Compute Module 4 devices for the DroneLeaf deployment image that is no longer distributed publicly. The image is provided as a Raspberry Pi Imager cache artifact so we can still apply the RPI imager built-in customization workflow.

## 1. Prerequisites

- Workstation running Ubuntu 20.04 or newer with sudo access. (Other OSes may work but are untested.)
- Raspberry Pi Imager (tested successfully with releases available as of Oct 2025).
- DroneLeaf `.cache` image pulled from OneDrive: https://droneleaf.sharepoint.com/:f:/s/technical/EgDEJFNFLIhEoZn-SytFl84BVfx-M386H0IUSG6FZXBCjw?e=ogItNg
- microSD card (Model B) or CM4 eMMC carrier with USB access.
- Reliable USB-C power supply and cables.

> The OneDrive image is the last cached Ubuntu 20.04 build captured from Raspberry Pi Imager before Canonical removed the download. Ubuntu `.img` or `.iso` releases from other sources will *not* accept Imager customizations, so use the `.cache` artifact supplied here.

## 2. Install Raspberry Pi Imager

1. Download Raspberry Pi Imager (consider older releases from the Raspberry Pi archive if the current version refuses the cache artifact).
2. Install the package as usual (deb, snap, or AppImage depending on your platform).
3. Launch Raspberry Pi Imager once so it creates its cache directories under `~/.cache/Raspberry Pi/Imager/`.

## 3. Register the Cached Image

1. Download the `.cache` file from the DroneLeaf OneDrive link above.
2. Copy the file into `~/.cache/Raspberry Pi/Imager/`. Keep the original filename so the Imager UI recognizes it in the “Operating System → Downloaded images” section.
3. Start Raspberry Pi Imager and select **Custom images** from the OS list, then pick the file you just downloaded [Make sure to select the option of show all files].
4. Continue to storage selection but do **not** click “Write” yet—we must apply customizations first.

> The benefit of the `.cache` file is that Raspberry Pi Imager treats it as an official download, unlocking the advanced customization dialog (hostname, SSH, Wi‑Fi, etc.).

## 4. Apply Customizations

1. Click the gear icon (Advanced Settings) before writing the image.
2. Configure the following:
   - Hostname: `dl-<loc>-<role>-<nnn>` #Check DroneLeaf_Networking_Configuration.md 
   - Enable SSH (password authentication is fine).
   - Username: `pi`
   - Password: `raspberry`
   - Configure Wi‑Fi:
     - SSID: `DroneLeaf_Deployment` #Example. Replace with the desired network name
     - Password: `kuri@1234!!` #Example. Replace with the desired network password
     - Wi‑Fi country: match the deployment locale.
   - Set local time zone and keyboard layout as needed.
3. Save the settings and return to the main screen.

## 5. Flash the Target Media

- **Raspberry Pi Model B**: Insert the microSD card, select it under “Storage,” then click **Write**. Safely eject the card when the process completes.
- **Compute Module 4 (CM4)**: Perform the flashing steps on the CM4’s eMMC as explained below.

## 6. CM4 USB Boot Preparation (eMMC Targets Only)

Install the USB boot helper once per workstation:

```bash
cd
sudo apt update
sudo apt install git libusb-1.0-0-dev pkg-config build-essential
git clone --recurse-submodules --shallow-submodules --depth=1 https://github.com/raspberrypi/usbboot
cd usbboot
make
```

> On the Pixhawk V6X CM4 carrier board, toggle the boot switch to **EMMC** before connecting the USB port. This exposes the CM4 eMMC storage to the host computer.
For other CM4 carriers, refer to the manufacturer’s instructions for enabling USB boot mode. (usually involves shorting specific pins or setting jumpers, or using a dedicated switch).

To load the eMMC as a USB mass storage device:

```bash
cd ~/usbboot
sudo ./rpiboot -l
```

After the eMMC appears, return to Raspberry Pi Imager, pick the detected “Compute Module” storage device, apply the customizations (Section 4), and write the image.

## 7. Extend the System Partition

> For CM4 eMMC targets, after flashing, the root filesystem partition may not utilize the full eMMC capacity. To fix this: 
1. Reconnect the CM4 in USB boot mode as described in Section 6.
2. Use `gparted` or similar partitioning tool to resize the root filesystem partition to fill the remaining space on the eMMC.
3. Safely eject the CM4 eMMC device when done.

## 8. Post-Flash Checks

1. Boot the device and connect it to the `DroneLeaf_Deployment` network (for CM4 on a lab desk, use Ethernet if available).
2. Verify SSH access using `ssh pi@DronLeaf_RaspberryPi`.
3. Confirm expected services or container workloads defined by the DroneLeaf image.

> Optionally, change the hostname to allow multiple devices on the same network without conflicts.
To change the hostname, edit `/etc/hostname` and `/etc/hosts` accordingly, then reboot.

You now have a repeatable flow for loading the legacy Ubuntu 20.04 DroneLeaf image onto Raspberry Pi Model B or CM4 hardware with full customization support.
