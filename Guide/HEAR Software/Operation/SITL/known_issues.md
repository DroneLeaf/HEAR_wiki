# Known issues — SITL & Bench

This page collects known issues encountered when running the HEAR SITL and bench environments. Each issue follows a consistent template so it's easy to scan, reproduce, and track status.

---

## Known issues (current)
### Variation in port number between SITL and Bench
- **ID:** KI-001
- **Title:** Variation in port number between SITL and Bench 
- **Affected components:** mavlink-router
- **Environment:** SITL | Bench | Both
- **Symptoms:** in LeafMC, you would not be able to connect to the vehicle
- **Workaround:** edit mavlink-router configuration file located at `/etc/mavlink-router/main.conf` and make sure you have the correct port number for your setup:
    - for SITL setup, use port `11000`
    - for Bench setup, use port `14540`
- **Status:** planned
- **Reported:** Nov 2025


### Switching between SITL and Bench profiles [No option in the web UI]
- **ID:** KI-002
- **Title:** Switching between SITL and Bench profiles [No option in the web UI]
- **Affected components:** DynamoDB, HEAR_FC
- **Environment:** SITL | Bench | Both
- **Symptoms:** No option in the web UI to switch between SITL and Bench profiles
- **Workaround:** Manually update the machine profile in DynamoDB using the web UI:
  1. Open the DynamoDB manager UI:
     - Config profiles: http://localhost:8080/table/config-profile?tabActive=search
     - Robot-instance assignments: http://localhost:8080/table/config-robot_instance_profile_assignment?tabActive=search

  2. Steps:
     - On the config-profile page, locate the row for "SITL" or "Bench-Testing" and copy the profile's id / address field you need.
     - Open config-robot_instance_profile_assignment, find the record for your machine, and update the `profile_id` with the copied value. Save.

  3. For more background on DynamoDB sync and HEAR_FC setup, see:
     [Guide/HEAR Software/Operation/SITL/DynamoDB-and-hearfc-debugging.md](Guide/HEAR Software/Operation/SITL/DynamoDB-and-hearfc-debugging.md)
- **Status:** Open
- **Reported:** Nov 2025

<!-- ## Issue template

Use the template below for each reported issue:

- **ID:** a short unique identifier (e.g. KI-001)
- **Title:** concise description
- **Affected components:** PX4 / Gazebo / LeafFC / LeafMC / mavlink-router / Petal App Manager / OS / Other
- **Environment:** SITL | Bench | Both
- **Severity:** Low / Medium / High / Critical
- **Symptoms:** short bullet list of observed behaviour
- **Reproduction steps:** numbered steps to reproduce
- **Logs / artifacts:** list of useful logs or files to collect (paths, commands)
- **Workaround:** temporary mitigation
- **Status:** Open / In progress / Fixed / Won't fix
- **Fix / PR:** link to the PR or commit that fixes the issue (when available)
- **Reported:** date and reporter

--- -->