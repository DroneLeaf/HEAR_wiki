# ZRAM Setup on Ubuntu 20.04 (Focal) — Practical, Fast, and Persistent

This guide shows a way to enable **zram-first swapping** on Ubuntu 20.04, with your regular NVMe `/swapfile` kept as a **low‑priority safety net**. It’s tuned for developer boxes (e.g., C++-heavy builds on 16 GB RAM).

> TL;DR: Install `zram-tools`, size zram to ~**1.5× RAM**, keep `/swapfile` at **lower priority**, and set `vm.swappiness=80`.

---

## 0 Quick checks (before you start)

```bash
swapon --show
zramctl || true
dpkg -l zram-tools || echo "zram-tools not installed"
````

If you only see `/swapfile`, proceed below — that's the default on 20.04.

## 1. Install

```bash
sudo apt update
sudo apt install -y zram-tools
````

This provides the `zramswap` systemd service and the config file `/etc/default/zramswap`.

## 2. Configure (`/etc/default/zramswap`)

Edit the file (`sudo nano /etc/default/zramswap`) and choose ONE sizing method:

- Recommended (percent of RAM):

```text
PERCENTAGE=150   # ~1.5× RAM (e.g. ~24 GiB on 16 GiB systems)
#ALLOCATION=
````

Also set:

```text
PRIORITY=100   # make zram preferred over the disk swapfile
# optionally:
ALGO=lz4       # low-latency compressor (if supported)
````

## 3. Activate zram and re-enable swap

```bash
sudo systemctl restart zramswap
sudo swapoff -a && sudo swapon -a
````

Running `swapoff -a && swapon -a` forces the kernel to prefer zram immediately if swap was already in use.

## 4. Keep `/swapfile` as a low-priority backstop

Edit `/etc/fstab` and set the swapfile priority lower (e.g., 10):

```text
/swapfile none swap sw,pri=10 0 0
````

Then re-enable it:

```bash
sudo swapoff /swapfile
sudo swapon -a
````

## 5. Recommended kernel tunings (optional)

Create `/etc/sysctl.d/99-buildbox.conf` with these values:

```text
# Prefer zram over disk swap
vm.swappiness=80

# Reduce latency when touching swap
vm.page-cluster=0

# Avoid sudden watermark boosts
vm.watermark_boost_factor=0
````

Apply and verify:

```bash
sudo sysctl --system
sysctl -n vm.swappiness vm.page-cluster vm.watermark_boost_factor
````

If `vm.swappiness` still reports `30`, a later-loading file (e.g., `/etc/sysctl.conf`) may be overriding it — place your file with a higher number suffix (e.g., `100-...`) to ensure it wins.

## 6. Verify

```bash
zramctl                   # shows zram devices & compressor
swapon --show --bytes     # shows active swap and priorities
````

What to expect:
- zram devices that add up to roughly your target (e.g., ~24 GiB on 16 GiB RAM)
- zram entries with PRI ≈ 100 (higher than the swapfile)
- `/swapfile` present with PRI 10 as a backstop

Example (good) outputs:

```text
NAME        ALGORITHM DISKSIZE  DATA  COMPR TOTAL STREAMS
/dev/zram0  lz4       1.4G      ...
...                                 # many devices -> ~24 GiB total

NAME        TYPE  SIZE   USED PRIO
/dev/zram0  partition 1.4G  0   100
/swapfile   file      31.7G 0    10
````

## 7. Troubleshooting (concise)

- No `/dev/zram*` listed:
  - Ensure `zram-tools` installed and `PERCENTAGE`/`ALLOCATION` are set in `/etc/default/zramswap`.
  - Restart: `sudo systemctl restart zramswap` then `sudo swapoff -a && sudo swapon -a`.

- zram appears in `zramctl` but not `swapon --show`:
  - Enable manually once: