# Planned Documentation for HEAR Software SITL Operation

This plan breaks the monolithic “SITL installation” doc into focused runbooks. Each filename below is fixed (per user requirement) and paired with scope, audience, cadence, and inter-doc links so contributors know what belongs where.

## Phase Map

| Phase | When to Read | Primary Docs |
| --- | --- | --- |
| 1. Provision hardware | New laptop or OS refresh | `development-machine-OS-installation-for-droneleaf-stack.md` |
| 2. Standardize tooling | After base OS is stable | `recommended-tools-and-common-practices.md`, `droneleaf-workspace-topology-and-repos-introduction.md` |
| 3. Install SITL & services | Anytime SITL stack is rebuilt | `sitl-installation-on-ubuntu20.04.md`, `SITL-drone-provisioning.md` |
| 4. Debug & UI tooling | Whenever data sync or UI tools change | `DynamoDB-and-hearfc-debugging.md`, `leafQGC-and-QT-tooling.md` |

## Document Blueprints

### development-machine-OS-installation-for-droneleaf-stack.md
- **Objective:** One-time checklist to turn bare hardware into a DroneLeaf-ready Ubuntu 20.04 box.
- **Audience & Trigger:** IT / engineers imaging a clean machine; run during laptop onboarding or full OS reinstall.
- **Key Sections:** BIOS/UEFI settings, disk sizing guidance, Ubuntu installer flow, post-install updates, hostname + swap configuration, region/time settings.
- **Dependencies & Links:** Feeds directly into `recommended-tools-and-common-practices.md` for next steps.

### recommended-tools-and-common-practices.md
- **Objective:** Canonical list of required utilities (Yakuake profile, logging approach, logging location, VS Code settings, git practices) to ensure consistency across all developer machines.
- **Audience & Trigger:** Every developer after OS install; revisit when tooling standards update.
- **Key Sections:** Shell/terminal tooling, package install list, log retention, naming conventions, “dos & don’ts.”
- **Dependencies & Links:** Link backwards to the OS install doc for prerequisites and forwards to `droneleaf-workspace-topology-and-repos-introduction.md` for repo-specific steps.

### droneleaf-workspace-topology-and-repos-introduction.md
- **Objective:** Explain the canonical directory tree (`~/software-stack`, `~/HEAR_CLI`, etc.), how repos relate, and which scripts manage them. It also lists common HEAR-related environment variables and services.
- **Audience & Trigger:** Anyone cloning repos for the first time or auditing directory drift.
- **Key Sections:** Workspace prerequisites, repo list + purpose, environment variables, storage requirements, cleanup tips.
- **Dependencies & Links:** Requires tooling from the previous doc; provides context referenced by both `sitl-installation-on-ubuntu20.04.md` and `leafQGC-and-QT-tooling.md`.

### sitl-installation-on-ubuntu20.04.md
- **Objective:** Idempotent runbook for installing the SITL stack via hear-cli and Docker on Ubuntu 20.04.
- **Audience & Trigger:** Devs rebuilding SITL, lab operators refreshing machines.
- **Key Sections:** hear-cli clone/update, staged installer commands (with log file naming), env certificate import, reboot cadence checklist, quick validation commands (`docker ps`, `systemctl status mavlink-router.service`).
- **Dependencies & Links:** Assumes directories from the topology doc; hands off to `SITL-drone-provisioning.md` once base services are running.

### SITL-drone-provisioning.md
- **Objective:** Describe the process to register a SITL instance as a “drone”: licensing, env cert sync, controller-dashboard prep, service verification.
- **Audience & Trigger:** Operations/support when bringing a new SITL node online or after major credential rotation.
- **Key Sections:** fly.droneleaf.io registration steps, env.zip handling, hear-cli init programs to run (with expected outputs), readiness checklist for containers/services.
- **Dependencies & Links:** Follows SITL installation; points readers to `DynamoDB-and-hearfc-debugging.md` for deeper troubleshooting if checks fail.

### DynamoDB-and-hearfc-debugging.md
- **Objective:** Pair DynamoDB sync verification with HEAR_FC build/debug instructions so data + FC issues are diagnosed together.
- **Audience & Trigger:** Engineers investigating data desync, FC build failures, or runtime anomalies.
- **Key Sections:** DynamoDB sync flow, HEAR_Msgs/HEAR_FC build commands, clean rebuild procedure, VS Code launch profiles, common error patterns.
- **Dependencies & Links:** References provisioning doc for prerequisites; links out to `SITL-drone-provisioning.md` for baseline readiness requirements.

### leafQGC-and-QT-tooling.md
- **Objective:** Self-contained guide for LeafMC clone, Qt 5.15.2 tooling, Qt Creator configuration, and GUI troubleshooting.
- **Audience & Trigger:** Developers working on LeafMC or needing QGroundControl integration.
- **Key Sections:** Repo clone instructions, hear-cli Qt installer usage, Qt Creator kit setup, license fixes, headless build tips.
- **Dependencies & Links:** Expects workspace structure from the topology doc and SITL services from the installation doc; optional reference to `recommended-tools-and-common-practices.md` for terminal setup.

## Implementation Notes
- Create each markdown file with a short front-matter block (purpose, audience, last-reviewed) before migrating content.
- Update existing entry points (`Guide/HEAR Software/Operation/SITL/Readme.md` and `SITL_Installation_and_preparation_on_new_machine.md`) to link to these new docs once content is moved.
- Preserve command ordering and reboot guidance verbatim when migrating so historical accuracy is maintained; only reorganize containers of text, not instructions themselves.
- add summary in each doc of expected outcomes and summary of instllation steps, or so. The purpose is if someone previously read the doc, they can quickly refresh their memory on what to expect, and insure steps were completed correctly.

