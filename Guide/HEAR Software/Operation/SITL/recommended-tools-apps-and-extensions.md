# Recommended Tools and Common Practices

Standardize day-to-day tooling so SITL instructions remain consistent across machines. Complete this checklist immediately after finishing the base OS installation.

## Terminal [Yakuake](../External%20Software/Yakuake/README.md)



## WireShark [with MAVLink support]
1. Install Wireshark:
   ```bash
   sudo apt install -y wireshark-qt
   ```
2. During installation, select **Yes** to allow non-superusers to capture packets.
3. Add your user to the `wireshark` group:
   ```bash
   sudo usermod -aG wireshark $USER
   ```
4. Log out and back in for group changes to take effect.
5. Use hear-cli to install MAVLink dissector:
   ```bash
   hear-cli local_machine run_program --p mavlink_update_wireshark_plugin 
   ```
   Note: if hear-cli is not yet installed, follow the instructions in the SITL installation guide to set it up first.
6. if you face permission issues, you can run with sudo:
   ```bash
   sudo wireshark
   ```

## Git and GitHub Access
1. As a reader of this guide, we assume you have access to the DroneLeaf GitHub organization. If not, request access from your team lead or the DevOps team.
2. Create a long-lived **personal access token (classic)**: [Fine grained tokens are not tested with HEAR ecosystem scripts and processes.]
   - *Token name:* `DroneLeaf Token`
   - *Resource owner:* DroneLeaf organization
   - *Expiration:* [No expiration]
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
   > Note: when using git in VS Code or other IDEs, you may be prompted to re-enter your token. This is expected behavior.



## Logging Best Practices
- **Logging:** When running scripted installers, suffix commands with `| tee <logname>.log` to capture output for support teams.


Continue with [`droneleaf-workspace-topology-and-repos-introduction.md`](droneleaf-workspace-topology-and-repos-introduction.md) once tooling is in place.
