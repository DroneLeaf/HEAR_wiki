# Installation Steps for Ubuntu 20.04 LTS

Set up a clean Ubuntu 20.04 LTS install that meets the minimum requirements for the DroneLeaf development stack.

## Prerequisites

- Bare-metal with at least 256 GB free disk space. [VM setups are not tested.]
- USB stick (≥8 GB) to host the Ubuntu installer. [In some cases, microSD cards may work. Follow the same process as USB drives.]
> Notes:
    Broadcom Wi-Fi cards on newer Intel laptops may need drivers post-install. Use Ethernet or a supported USB Wi-Fi adapter to continue.

## OS setup steps

1. Back up all important data. If you plan to dual-boot, ensure a partition with ≥200 GB free (512 GB recommended). Shrink artitions from Windows using Disk Management or follow <https://help.ubuntu.com/community/HowtoResizeWindowsPartitions>.
2. Download the Ubuntu 20.04 ISO from the official site and create a bootable USB using Rufus or Etcher: <https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview>.
3. Insert the USB in the target machine and boot from it:
   - Restart and press the BIOS/UEFI key (F2, F12, DEL, ESC depending on vendor). [Do not shut down and cold boot; use restart. Hibernation may cause issues.]
   - It is recommended to disable Secure Boot.
   - Set the USB drive as the first boot device, save, and exit.
4. Choose **Try Ubuntu** to enter the live environment before installing.
5. Double-click **Install Ubuntu** to begin.
6. Follow the installer prompts:
   - Choose language and keyboard layout.
   - Select **Normal installation** and enable both *Download updates while installing* and *Install third-party software…*.
   - For dual-booting, shrink the existing partition first [if not done previously] (GParted or Windows Disk Management) and select **Install Ubuntu alongside...**.
   - To dedicate the entire disk, pick **Erase disk and install Ubuntu** (destroys existing data). [recommended for fresh installs.]
7. Set time zone, create the `droneleaf` user, and record the password for documentation parity.
8. It is recommended to use username `droneleaf` to match documentation references.
9. continue installation until complete.

## Post-install configuration steps
1. Update the base system:
    ```bash
    sudo apt update
    sudo apt upgrade -y
    ```
2. Install proprietary drivers if required via **Software & Updates → Additional Drivers**.
3. Do **not** upgrade to 22.04 or later; the DroneLeaf stack currently supports 20.04 only.
4. Align locale settings used in documentation:
    - **Settings → Region & Language → Formats → United Kingdom**.
    > prevents any unexpected issues with non-ASCII characters in code samples.
5. Rename the host to `dl-dev-SITL-xx` for clarity in shared docs, replacing `xx` with your assigned ID:
    ```bash
    sudo hostnamectl set-hostname dl-dev-SITL-xx
    ```
6. Add a 16 GB swap file to stabilize large builds:
    ```bash
    free -h
    sudo fallocate -l 16G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    sudo cp /etc/fstab /etc/fstab.backup
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    ```
16. Optional: use zram for additional memory compression: [zram_installation_and_configuration.md](zram_installation_and_configuration.md).

17. Using 2020+ laptop hardware (Intel 11th gen or newer)? Perform the kernel upgrade steps at [kernel_upgrade_for_2020_plus_chipset.md](kernel_upgrade_for_2020_plus_chipset.md) to ensure all device drivers function correctly. 

quick checklist
--------------------
- Ubuntu 20.04 LTS installed.
- 256 GB recommended free disk space remains.
- swap and zram configured.
- apt packages updated and upgraded.
