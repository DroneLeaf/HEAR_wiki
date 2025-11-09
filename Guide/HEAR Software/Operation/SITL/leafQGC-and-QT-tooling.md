# LeafQGC and Qt Tooling

LeafQGC (a.k.a. LeafMC) is DroneLeaf’s fork of QGroundControl. Follow this guide to prepare Qt dependencies, clone the repo, and resolve common IDE issues.

## Clone LeafMC

```bash
cd ~
git clone --recurse-submodules -j8 -b dev-sitl git@github.com:DroneLeaf/LeafMC.git
cd LeafMC
sudo ./tools/install-dependencies-debian.sh
```

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

1. Open Qt Creator and load `~/LeafMC/CMakeLists.txt`.
2. Navigate to **Tools → Options → Kits → Qt Versions**:
   - Remove all Qt versions except **5.15.2**.
   - If absent, click **Add** and point to `~/Qt/5.15.2/gcc_64/bin/qmake`.
3. Switch the active kit to use Qt 5.15.2 and click **Apply**.
4. On the **Projects** page, adjust the **Build Environment** so the `PATH` begins with:
   ```bash
   PATH=$HOME/Qt/5.15.2/gcc_64/bin:/usr/bin:$HOME/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:/usr/games:/usr/local/games:/snap/bin
   ```
5. Click **Configure Project** to generate build files.

### License Check Failures

If Qt Creator shows **License check failed, Giving up.**:

1. Go to **Tools → Options → License**.
2. Click **Add → I have a license key file**.
3. Select `~/Qt/Tools/QtCreator/licenses/qt-creator-license.xml`.
4. Restart Qt Creator and rebuild.

You can now build and run LeafMC directly from Qt Creator or via command line using `cmake --build build-directory`.
