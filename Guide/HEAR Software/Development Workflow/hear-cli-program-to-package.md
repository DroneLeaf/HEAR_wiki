# HEAR_CLI Programs as Debian Packages

### Overview:

HEAR_CLI programs are distributed as standalone Debian packages (.deb) to allow:

* Installation via APT repositories (staging / production)

* Versioned releases per program

* Architecture-specific builds (amd64, arm64)

* Controlled promotion between environments

* Elimination the need of repository cloning on target machines (SITL)

Each HEAR_CLI program is packaged independently and installed under the DroneLeaf user directory structure.


---

### Workflow Name:

```
HEAR_CLI program to package
```

Triggered manually via **workflow_dispatch**

---

### Trigger Inputs:

| Input          | Required | Description                               |
| -------------- | -------- | ----------------------------------------- |
| `program_name` | Yes      | Folder name under `scripts/programs/`     |
| `version`      | Yes      | Debian package version                    |
| `architecture` | Yes      | `amd64` or `arm64`                        |
| `environment`  | Yes      | Target APT repo (`staging`, `production`) |
| `branch`       | Yes      | Git branch to build from                  |
| `notes`        | No       | Release notes (included in summary only)  |


---

### High-Level Build Flow:

1. Checkout:

HEAR_CLI main Branch.

2. Package Name Normalization:

convert "_" into "-" as a publish name.

3. Validation:

The workflow validates that required inputs are provided and Program exists at:
```
scripts/programs/<program_name>/
```

If not found, the build fails.

4. Package Directory Structure:

The following Debian structure is generated dynamically:
```
package/
├── DEBIAN/
│   ├── control
│   ├── preinst
│   ├── postinst
│   └── prerm
└── home/.droneleaf/hear_cli/programs/<program_name>/

```
**important:** Programs are installed under:
```
/home/.droneleaf/hear_cli/programs/<program_name>
```
This matches HEAR_CLI runtime expectations.

5. Control File Generation:

Control file is generated dynamically. example:
```
Package: drone-setup-tool
Version: 1.2.0
Architecture: amd64
Section: base
Priority: optional
Description: HEAR_CLI program drone_setup_tool
```
No explicit Depends field is currently defined.

6. Installation Scripts:

The following scripts are generated:
* preinst
* postinst
* prerm

They are placeholders for future lifecycle logic.

7. Program File Copy:

All files from:
```
scripts/programs/<program_name>/
```
Are copied into:
```
package/home/.droneleaf/hear_cli/programs/<program_name>/
```

8. Dynamic Function Resolution:

The workflow parses main.sh of the program and extracted function script paths are copied from:
```
scripts/<function_path>
```
into:
```
programs/<program_name>/functions/
```

This ensures runtime completeness of the program package.

9. Permission Setup.

10. Build Step:

using:
```
dpkg-deb --build package
```
Then renamed to:
```
<publish_name>_<version>_<architecture>.deb
```

11. Artifact Upload:

The generated .deb is uploaded as a GitHub Actions artifact.

---

### Publish Job:

Runs after successful build.

1. Download Artifact:

Downloads previously built .deb.

2. Publish to APT Repository:

Uses internal composite action:

```
.github/actions/publish-apt
```

Inputs:

- package_path
- repository_username
- repository_password
- environment (staging / production)

This pushes the package to the selected APT repository.

---

### Installation (Client Side):

Once published:

```
sudo apt update
sudo apt install <package-name>
```

---

### Architecture Handling:

The package is marked as:
* amd64
* arm64

The build runner itself is ubuntu-latest (x86).

The architecture field is declarative and assumes compatibility of the packaged scripts.

Since HEAR_CLI programs are shell-based, cross-compilation is not required.

---

### Design Constraints:

* Installation path is inside /home/.droneleaf.
* No Python virtual environments are packaged.
* No system-level binaries are placed in /usr/bin.

---

### Intended Outcome:

After publishing:

* DroneLeaf teams can install a specific HEAR_CLI program via APT.
* Program is versioned and environment-controlled.
* No manual file copying required.
* No manual run from HEAR_CLI.
* Promotion from staging to production is controlled via workflow input.

---

### Definition of Done:

* .deb builds successfully.
* Installs without dpkg errors.
* Files land in expected directory.
* Program executes correctly through HEAR_CLI when needed.
* Package is visible in selected APT repository.

---

### Maintainer:

Hashem Allaham <hashem.allaham@droneleaf.io>