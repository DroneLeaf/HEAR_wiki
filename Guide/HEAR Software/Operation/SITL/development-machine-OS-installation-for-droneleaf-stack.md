# Development Machine OS Installation for DroneLeaf Stack

Set up a clean Ubuntu 20.04 LTS install that meets the minimum requirements for the DroneLeaf SITL stack. Perform these steps whenever you provision brand-new hardware or need to reimage an existing workstation.

<!-- provide summary of the installation process and key points -->

## Prerequisites

- Bare-metal with at least 256 GB free disk space. [VM setups are not tested.]
- Reliable internet connection for package downloads.
- USB stick (≥8 GB) to host the Ubuntu installer. [In some cases, microSD cards may work. Follow the same process as USB drives.]

## Installation Steps for Ubuntu 20.04 LTS

1. Back up all important data. If you plan to dual-boot, ensure a partition with ≥200 GB free (512 GB recommended). Shrink partitions from Windows using Disk Management or follow <https://help.ubuntu.com/community/HowtoResizeWindowsPartitions>.
2. Download the Ubuntu 20.04 ISO from the official site and create a bootable USB using Rufus or Etcher: <https://ubuntu.com/tutorials/create-a-usb-stick-on-ubuntu#1-overview>.
3. Insert the USB in the target machine and boot from it:
   - Restart and press the BIOS/UEFI key (F2, F12, DEL, ESC depending on vendor). [Do not shut down and cold boot; use restart. Hibernation may cause issues.]
   - It is recommended to disable Secure Boot.
   - Set the USB drive as the first boot device, save, and exit.
4. Choose **Try Ubuntu** to enter the live environment before installing.
5. Notes:
   - Broadcom Wi-Fi cards on newer Intel laptops may need drivers post-install. Use Ethernet or a supported USB Wi-Fi adapter to continue. Alternatively, replace the internal network card with a supported model [intel 8th gen or later]
   - Plan for a kernel update later to pick up the latest drivers. 
6. Double-click **Install Ubuntu** to begin.
7. Follow the installer prompts:
   - Choose language and keyboard layout.
   - Select **Normal installation** and enable both *Download updates while installing* and *Install third-party software…*.
   - For dual-booting, shrink the existing partition first [if not done previously] (GParted or Windows Disk Management) and select **Install Ubuntu alongside...**.
   - To dedicate the entire disk, pick **Erase disk and install Ubuntu** (destroys existing data). [recommended for fresh installs.]
8. Set time zone, create the `droneleaf` user, and record the password for documentation parity.
9. continue installation until complete.
10. Log in with the account created earlier.
11. Update the base system:
    ```bash
    sudo apt update
    sudo apt upgrade -y
    ```
12. Install proprietary drivers if required via **Software & Updates → Additional Drivers**.
13. Do **not** upgrade to 22.04 or later; the DroneLeaf stack currently supports 20.04 only.
14. Align locale settings used in documentation:
    - **Settings → Region & Language → Formats → United Kingdom**.
15. Rename the host to `dl-dev-SITL-xx` for clarity in shared docs, replacing `xx` with your assigned ID:
    ```bash
    sudo hostnamectl set-hostname dl-dev-SITL-xx
    ```
16. Add a 16 GB swap file to stabilize large builds:
    ```bash
    free -h
    sudo fallocate -l 16G /swapfile
    sudo chmod 600 /swapfile
    sudo mkswap /swapfile
    sudo swapon /swapfile
    sudo cp /etc/fstab /etc/fstab.backup
    echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
    ```
17. Optional: use zram for additional memory compression:
    ```bash
    sudo apt install -y zram-tools
    ```
    follow configuration instructions at zram_installation_and_configuration.md

18. Are you using 2020+ laptop hardware (Intel 11th gen or newer)? If so, perform the kernel upgrade steps at [post_2020_laptop_driver_fix_by_kernel_upgrade.md](post_2020_laptop_driver_fix_by_kernel_upgrade.md) to ensure all device drivers function correctly.

quick checklist
--------------------
- Ubuntu 20.04 LTS installed.
- 256 GB recommended free disk space remains.
- Yakuake configured and ready.
- [optional] Username is `droneleaf` to match documentation references.
- swap and zram configured.
- apt packages updated and upgraded.

✓ Continue with [`recommended-tools-and-common-practices.md`](recommended-tools-and-common-practices.md) once the base OS is stable.
