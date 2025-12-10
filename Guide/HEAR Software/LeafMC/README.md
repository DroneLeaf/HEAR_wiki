# LeafMC and Qt Tooling

LeafMC is DroneLeaf’s fork of QGroundControl. Follow this guide to prepare Qt dependencies, clone the repo, and resolve common IDE issues.

## Clone LeafMC 
cloning of LeafMC is covered in HEAR Software Getting Started guide. You can locate the cloned repo at `~/software-stack/LeafMC`.


> Use the `dev-sitl` branch for the latest SITL-focused development build unless instructed otherwise.

## Qt 5.15.2 Requirement

1. Check the installed Qt version:
   ```bash
   qmake --version
   ```
2. If 5.15.2 is missing, install it via hear-cli:
   ```bash
   hear-cli local_machine run_program --p install_qt
   ```
3. Install Qt Creator:
   ```bash
   sudo apt-get install qtcreator
   ```

## Configure Qt Creator

1. Open Qt Creator and load `~/software-stack/LeafMC/CMakeLists.txt`.
2. Navigate to **Tools → Options → Kits → Qt Versions**:
   - Remove all Qt versions except **5.15.2**.
   - If absent, click **Add** and point to `~/Qt/5.15.2/gcc_64/bin/qmake`.
3. Switch the active kit to use Qt 5.15.2 and click **Apply**.
4. On the **Projects** page, adjust the **Build Environment** so the `PATH` begins with:
   ```bash
   PATH=$HOME/Qt/5.15.2/gcc_64/bin:/usr/bin:$HOME/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin
   ```
5. Click **Configure Project** to generate build files.

## Common Qt Creator Issues
- **Missing qmake or wrong version**: Ensure Qt 5.15.2 is installed and configured as above.
- **Wrong starting file**: Set the main project file to `CMakeLists.txt` in the project tree.
- **Missing SDL2**: Install SDL2 development package:
   ```bash
   sudo apt-get install libsdl2-dev
   ```

## LeafQGC common connection issues
- There are two level of connection, both handeled by the mavlink-router service:
      - Connection between QGC and PX4 SITL (MAVLink connection)
      - Connection between QGC and HEAR_FC (MAVLink connection)
- you might have the first connection working but not the second one [Identify by the red endicator on top left of LeafQGC screen]
- Your mavlink-router service might not be properly forwarding the mavlink messages between PX4 and HEAR_FC. Note that there are two possible drone setup: bench [external] and SITL [usually internal]. Make sure you have the correct setup on your mavlink-router configuration file located at:  `/etc/mavlink-router/main.conf`
- Make sure that both PX4 SITL and HEAR_FC are running.

## Getting started with LeafQGC
- With mavlink router and PX4 SITL running, launch LeafQGC from Qt Creator or use the AppImage from [software-stack repository](https://github.com/DroneLeaf/software-stack/releases).
- Open `App Settings → Comm Links` and connect to `TCP://:5760` if not auto-connected. 
> DroneLeaf is intending on staching the TCP 5760 connection and use UDP 15760 instead in future releases.
- The vehicle should appear in the UI shortly after connection.
- If LeafFC is running, status would be ready for takeoff. with green indicators.