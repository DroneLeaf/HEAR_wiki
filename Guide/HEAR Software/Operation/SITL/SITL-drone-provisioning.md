# SITL Drone Provisioning

After Docker and system dependencies are installed, provision the SITL node so it can pull private images, sync data, and appear as a registered drone in DroneLeaf services.

## Prerequisites

- Completed `sitl-installation-on-ubuntu20.04.md`
  ``` 
  check the drone dashboard is accessible at http://localhost 
  > Note: if provisioning on https://fly.droneleaf.io is not completed, it is normal for leafFC.service to be stopped. [which you this guide would cover next]

## 4. Licensing and Data Sync

To finish provisioning, bind the SITL node to a drone license and sync DynamoDB data:
> as of Nov 2025, you need to start with a bench to be able to complete the provisioning steps. After licensing the bench, check the next note:

> you can switch between bench and SITL modes by following the instructions in [Moving_between_SITL_PX4_Autopilot_and_physical_pixhauk.md](../Hardware%20and%20Process/Commissioning/Moving_between_SITL_PX4_Autopilot_and_physical_pixhauk.md)

1. Register on <https://fly.droneleaf.io> and bind the SITL hardware to the assigned license following the flowchart in [Guide/Hardware and Process/Commissioning/2 Ready for FSAC/Ready for FSAC Updated.draw.io](./../../../Hardware%20and%20Process/Commissioning/2%20Ready%20for%20FSAC/Ready%20for%20FSAC_updated.drawio). Ensure the license status is active before pulling data.

> Note: as of Nov 2025, the fly.droneleaf.io portal doesn't allow saved templates, and you have to create a new of every document type while you go through the commissioning flow.

2. Once licensed, proceed to [`DynamoDB-and-hearfc-debugging.md`](DynamoDB-and-hearfc-debugging.md) to run the DynamoDB sync, build `HEAR_Msgs`, and prepare the HEAR_FC workspace.

At this point the SITL workstation is recognized as a drone and ready for functional testing.