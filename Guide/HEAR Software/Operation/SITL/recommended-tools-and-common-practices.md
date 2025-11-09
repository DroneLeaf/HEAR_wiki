# Recommended Tools and Common Practices

Standardize day-to-day tooling so SITL instructions remain consistent across machines. Complete this checklist immediately after finishing the base OS installation.

## Terminal Workflow

- Use **Yakuake** as the default drop-down terminal (`F12`). Rename each tab according to the task (e.g., `env_setup`, `hear_docker_clone`) so logs are easy to match with documentation.
- Enable unlimited scrollback (`Settings → Configure Profiles → Scrolling`) to preserve command history for audits.
- Capture long-running installer output with `tee` and store logs in the home directory (see examples in the SITL installation guide).

## Base Packages

Install the baseline developer packages in a dedicated Yakuake tab named `env_setup`:

```bash
sudo apt install -y build-essential libdbus-glib-1-dev libgirepository1.0-dev \
    git curl wget cmake unzip pkg-config libssl-dev libjpeg-dev libpng-dev \
    libtiff-dev libusb-1.0-0-dev python3-pip jq
```

## Git and GitHub Access

1. Sign in to <https://github.com> using Firefox (preinstalled) and make sure you have access to the private repositories (`DroneLeaf/HEAR_CLI`, etc.).
2. Create a long-lived **personal access token (classic)**:
   - *Token name:* `DroneLeaf Token`
   - *Resource owner:* DroneLeaf org (if applicable) otherwise your account
   - *Expiration:* 366 days
   - *Repository access:* All repositories
   - *Permissions → Repository → Contents:* Read and write
3. Configure Git globally (replace placeholders):
   ```bash
   git config --global user.email "your_email@example.com"
   git config --global user.name "Your Name"
   ```
4. Cache the GitHub token so CLI scripts can authenticate:
   ```bash
   git config --global credential.helper 'cache --timeout=3600'
   git credential approve <<EOF
   protocol=https
   host=github.com
   username=your_github_username
   password=your_personal_access_token
   EOF
   ```
   > Tip: Keep the GitHub page with the generated token open until you complete these steps; you cannot view it again afterward.

## Operational Conventions

- **Logging:** When running scripted installers, suffix commands with `| tee <logname>.log` to capture output for support teams.
- **Reboots:** Honor every reboot instruction in the SITL installation guide; services such as Docker socket configuration and systemd autostart rely on it.
- **Network hygiene:** Use wired connections during bulk installs to avoid Wi-Fi driver hiccups. Document any driver deviations in team notes.
- **Security:** Never copy tokens or certificates into shared directories. Use your home directory and tighten permissions when extracting sensitive files.

Continue with [`droneleaf-workspace-topology-and-repos-introduction.md`](droneleaf-workspace-topology-and-repos-introduction.md) once tooling is in place.
