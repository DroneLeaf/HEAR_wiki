Provisioning steps — SITL / First Run

> as per Nov 2025, provisioning is only possible on bench environment. After provisioning the bench, you can switch to SITL following [Moving_between_SITL_PX4_Autopilot_and_physical_pixhauk.md](./Moving_between_SITL_PX4_Autopilot_and_physical_pixhawk.md)

0. Power up your pixhawk PX6 and connect it to your development machine via Ethernet cable. Set your machine's Ethernet interface to static IP at 192.168.144.6/24.

Follow the steps as per the flow chart at: [Guide/Hardware and Process/Commissioning/2 Ready for FSAC/Ready for FSAC Updated.draw.io](../../../Hardware%20and%20Process/Commissioning/2%20Ready%20for%20FSAC/Ready%20for%20FSAC%20Updated.draw.io). Ensure the license status is active before pulling data.


1. Start MAVLink
    - Launch your MAVLink endpoint/service (example):
      - mavlink-router: sudo systemctl start mavlink-router

2. Start PX4 (SITL)

3. Start LeafFC

4. Open the local provisioning UI
    - Open your browser to localhost where the Leaf provisioning UI runs (e.g. http://localhost).
    - The UI should display a link/button to "Provision with Fly".

5. Follow the Fly link
    - Click/open the link to https://fly.droneleaf.io in your browser.
    
6. Sign in / Sign up on fly.droneleaf.io
    - Authenticate with your Leaf/Fly account or create one.

7. Add a new drone
    - In fly.droneleaf.io, choose "Add Drone" (or similar).
    - When prompted for a machine id, copy the Machine ID shown in your local UI or from your device:
      - Example local UI field: MACHINE ID: <your-machine-id>
      - Copy and paste that Machine ID into the Fly add-drone form.

8. Get API key
    - After adding the drone, Fly will generate an API key for that machine.
    - Copy the API key.

9. Paste API key back into localhost
    - Return to the local provisioning UI at http://localhost.
    - Paste the API key into the API key field.

10. Complete provisioning locally
     - Click "Next" / "Complete" in the local UI to finalize provisioning.
     - The UI should confirm successful connection and provisioning.

11. Run Petal App Manager
     - Start the Petal App Manager as required (see official docs for exact commands) at [link](https://droneleaf.github.io/petal-app-manager/getting_started/quickstart.html).

12. Finalize provisioning on fly.droneleaf.io
     - Back on fly.droneleaf.io, confirm the drone appears online and complete any remaining setup steps (naming, capabilities, metadata).
     - Verify telemetry and controls are functional.

13. Switch to SITL mode (if applicable)
     - If you provisioned on bench mode, follow instructions in [Moving_between_SITL_PX4_Autopilot_and_physical_pixhauk.md](./Moving_between_SITL_PX4_Autopilot_and_physical_pixhawk.md) to switch to SITL mode.    
