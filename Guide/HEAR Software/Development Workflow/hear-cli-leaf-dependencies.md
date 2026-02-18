# HEAR_CLI Leaf Dependencies Sub-Packages – CI/CD Documentation

### Overview:
This workflow ensures reproducible OS-level dependency preparation across ORIN and SITL environments and centralizes DroneLeaf dependency management.

This workflow builds and publishes a set of .deb sub-packages such as:
- leaf_mavlink
- leaf_opencv
- leaf_ros_neotic
- leaf_node
- leaf_redis
- leaf_cmake
- leaf_vcpkg
- leaf_cpr
- leaf_cpp_terminal
- leaf_onnxruntime
- leaf_docker

**The workflow supports:**

- Multi-target builds (ORIN, SITL)
- Manual version selection
- Publishing to staging or production APT repositories

---

### Repository Structure & Sources:

All package preparation scripts and related assets reside in:

Repository: `HEAR_CLI`

And the actual .deb packaging is performed via UPM (Universal Package Manager).

---

### Packaging Engine:

The workflow uses:

- UPM repository (cloned dynamically)
- upm.yaml for package configuration
- upm_cli package <package> <profile> deb

**Profiles used:**
- orin_release
- sitl_release

Each profile produces a target-specific .deb package.

---

### Workflow Location:
The workflow file resides in:

```
.github/workflows/leaf-dependencies-sub-packages.yml
```

Workflow Name:

```
Leaf Dependencies Sub-Packages
```

---

### How to Trigger the Workflow:

The workflow is manually triggered via GitHub Actions.

#### Steps:

1. Go to the GitHub repository.
2. Navigate to Actions tab.
3. Select Leaf Dependencies Sub-Packages.
4. Click Run workflow.

#### required Inputs:

1. **Package Name:** Available options:

* leaf_mavlink
* leaf_ros_neotic
* leaf_opencv
* leaf_node
* leaf_rapidjson
* leaf_redis
* leaf_cmake
* leaf_vcpkg
* leaf_cpr
* leaf_cpp_terminal
* leaf_onnxruntime
* leaf_docker

2. **Package Version:** Must match the desired version to publish.

Example: 
```
2.0.1
```
Ensure the version does not already exist in the target APT repository.

3. **Environment (APT Repository):**

Options:

- apt_staging
- apt (production)

Important: Always publish to apt_staging first. Promote to apt only after validation.

4. **Release Notes (Optional):**

Used for documentation and tracking.

---

### The Build Process:

The workflow performs:

1. **Checkout Repository:** Checks out current repository (contains packaging configuration).
2. **Clone Dependencies:** Clones HEAR_CLI and UPM.
3. **Configure UPM:** Modifies upm.yaml dynamically, Sets workspace path and package version.
4. **Install UPM Dependencies:** Installs required Python dependencies for packaging.
5. **Build .deb Package:** For each target: ORIN & SITL
it runs:
```bash
./upm_cli package <package_name> <profile> deb
```
Artifacts are generated under:
```
UPM/dist/
```

---

### The Publishing Process:

After successful build:

1. Artifact is uploaded.
2. Publish job downloads artifact.
3. Custom GitHub Action .github/actions/publish-apt:
   - Authenticates to APT repository
   - Uploads .deb
   - Publishes to selected environment (apt_staging or apt)

**Credentials required:**

- repository_username
- repository_password

Configured as GitHub secrets.

---

### Targets Matrix:

Each package is built twice:

| Target | Profile      | Purpose                 |
| ------ | ------------ | ----------------------- |
| ORIN   | orin_release | Edge device deployment  |
| SITL   | sitl_release | Simulation environments |

---

### Output Naming Convention:

```
<package-name>-<version>-<TARGET>.deb
```
Example:
```
leaf-mavlink-1.0.3-ORIN-Release.deb
```

Note:

Underscores in package names are automatically converted to dashes.

---

### Promotion Strategy:

Recommended release flow:

1. Build and publish to apt_staging.
2. Validate installation on:
   - ORIN device
   - SITL environment
3. If validated, rerun workflow with:
   - Same version
   - environment = apt

---

### How to Add a New Sub-Package:

1. Add installation logic to HEAR_CLI.
2. Define package entry in UPM/upm.yaml.
3. Add package option to workflow package_name input.
4. Validate build locally (if possible).
5. Trigger workflow.


---

### Troubleshooting:

**Build Failure:**

Check:
* upm.yaml configuration
* Version conflicts
* Missing workspace paths

**Publish Failure:**
Check:
* APT repository credentials
* Repository availability
* Version already exists

---

### Responsibilities:

- DevOps owns CI/CD pipeline and APT publishing.
- Feature owners maintain installation scripts inside HEAR_CLI.
- Release validation required before publishing to production APT.

---

### Maintainer:

Hashem Allaham <hashem.allaham@droneleaf.io>

