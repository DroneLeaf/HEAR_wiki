# Provisioning steps — First Run (Bench or SITL)

> Note (Nov 2025): provisioning is currently supported only on the bench environment. After provisioning a bench device you can switch to SITL — see the switching guide below.[Moving_between_SITL_PX4_Autopilot_and_physical_pixhauk.md](./Moving_between_SITL_PX4_Autopilot_and_physical_pixhawk.md)

Follow these steps to provision a device locally and connect it with Fly (https://fly.droneleaf.io).


## Provisioning steps
### Initial Steps related to development environment setup:

1. Power up your Pixhawk PX6 and connect it to your development machine via Ethernet.
2. Configure your machine's Ethernet interface with a static IP (example: `192.168.144.6/24`).


3. Start MAVLink
    ```bash
    sudo systemctl start mavlink-router.service
    ```
### Common Steps 
- Follow the flowchart for the full process: [Ready for FSAC flowchart](./../../../Hardware%20and%20Process/Commissioning/2%20Ready%20for%20FSAC/Ready%20for%20FSAC_updated.drawio).


4. Open the local provisioning UI

    Open a browser on the machine hosting the local provisioning UI (for example: http://localhost in development or edge device IP in production). The UI should show a "Provision with Fly" button or link.

5. Follow the Fly link

    Click the link to https://fly.droneleaf.io to open the Fly web app.

6. Sign in / Sign up on Fly

    Authenticate using your Fly/Leaf account. Create an account if you do not already have one.

7. Add a new drone in Fly

    - In Fly, choose "Add Drone" (or similar).
    - When asked for the Machine ID, copy the Machine ID shown in the local provisioning UI (example field: `MACHINE ID: <your-machine-id>`) and paste it into Fly's add-drone form.

8. Retrieve the API key

    After adding the drone, Fly will generate an API key for that machine. Copy the API key provided by Fly.

9. Paste the API key into the local UI

    Return to the local provisioning UI (e.g., http://localhost) and paste the API key into the API key field.

10. Complete provisioning locally

    Click "Next" / "Complete" in the local UI to finalize provisioning. The UI should confirm successful provisioning and connection.

11. Start Petal App Manager

    Launch the Petal App Manager per its quickstart docs: https://droneleaf.github.io/petal-app-manager/getting_started/quickstart.html

12. Finalize provisioning on fly.droneleaf.io
     - Back on fly.droneleaf.io, confirm the drone appears online and complete any remaining setup steps (naming, capabilities, metadata).
     - Verify telemetry and controls are functional.

13. (Optional) Switch to SITL

    If you provisioned a bench device and want to switch to SITL, follow the switching guide:

    [./Moving_between_SITL_PX4_Autopilot_and_physical_pixhawk.md](./Moving_between_SITL_PX4_Autopilot_and_physical_pixhawk.md)

If anything fails during provisioning, capture logs from the local UI and mavlink-router and consult the repository troubleshooting docs.