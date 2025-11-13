## Fix for 2020+ Intel laptop driver issues (kernel upgrade)

Summary
-------
Installation of newest kernel versions (>=6) and Intel microcode updates to resolve device driver issues on laptops with 11th gen Intel or newer CPUs. 


Steps
--------------------------
### 1. Update package lists and upgrade existing packages    
```bash
sudo apt update
sudo apt upgrade -y
``` 

### 2. Use the ubuntu-mainline-kernel.sh helper:

```bash
wget https://raw.githubusercontent.com/pimlie/ubuntu-mainline-kernel.sh/master/ubuntu-mainline-kernel.sh
chmod +x ubuntu-mainline-kernel.sh
# show available kernels
./ubuntu-mainline-kernel.sh -r
# install latest stable mainline kernel (example)
sudo ./ubuntu-mainline-kernel.sh -i v5.11.0
sudo reboot
```

After reboot, verify:

```bash
uname -r
# expected: 5.11.x or the version you installed
```

Intel microcode (recommended)
----------------------------
Install Intel microcode packages so CPU errata are handled correctly by the kernel:

```bash
sudo add-apt-repository ppa:intel-microcode/ppa -y
sudo apt update
sudo apt install intel-microcode -y
sudo reboot
```

Verify microcode applied:

```bash
dmesg | grep -i microcode
# look for lines like: "microcode: CPU0 sig=... revision=..."
```


Rollback / recovery
-------------------
If the new kernel causes problems, you can boot the previous kernel from the GRUB menu at boot time (Advanced options → previous kernel). To permanently roll back:

```bash
sudo apt remove --purge linux-image-<version-you-installed>
sudo update-grub
```

If issue persists, use installation media to boot into a live environment, mount the root filesystem, chroot, and remove the problematic kernel package as above.