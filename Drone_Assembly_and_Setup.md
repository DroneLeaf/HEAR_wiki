# Drone Assembly and Setup

Welcome to the Drone Assembly and Setup guide for **DroneLeaf's** workshop at IROS 2024. In this guide, you will learn how to assemble and configure your custom-built drone for a seamless demonstration experience during the workshop.

**This is an example of a drone you could build and fly today!**
![X500 Top View](https://github.com/user-attachments/assets/7738ae55-95e4-476b-89f6-1fb5823b29f9)

(revise the photo for cuteness)
## Components Required:
-   X500 Drone Frame with Raspberry Pi and Pixhawk Flight Controller Attached
-   Propulsion system (choose based on preference) 
	- Option A: TF40Pro Tri-blade
	- Option B: TF40Pro Twin-blade
	- Option C: Hubro Twin-blade
-   Payload (choose based on preference)
	- Option A: Lidar
	- Option B: Gimble
- Battery (choose based on preference)
	- Option A: MANIAX 5100
	- Option B: FullyMax 4400
-   Radio Frequency Transmitter (for kill-switch)
-   Laptop/PC (for mission control)

(Integrate with storage experience)
(Registration is in the first step and laptops are in storage)
## Assembly Instructions:
(We should talk about safety)
1.  **Gather All Components**: Ensure you have all the listed components available.
2.  **Attach the Motors**: Match the correct direction of rotation (CW or CCW) by comparing the label on the motor with the arrows found on the X500 frame. Attach the motors to the X500 frame using the four M4 screws provided, the long screws go in the back while the short screws go in the front (See image for clarity) . 

(Highlight the subject in images)

(Add figure numbers and captions for referencing)

(Better to upgrade to GIF)
![CounterClockwise Symbol](https://github.com/user-attachments/assets/99320337-4b50-41fa-9744-42186441ea9a)
![CCW Motor](https://github.com/user-attachments/assets/7bc79d0f-2000-4734-8306-5c3ffdd504dc)

3.  **Connect the Motors**: The motor cables are color codded. Connect the motor cables to the Electronic Speed Controllers (ESCs) by connecting the similarly colored cables together. (Three cables per motor)
![Color Coded Cables](https://github.com/user-attachments/assets/048a7a8e-9fbf-40eb-83cf-1a4c0e059516)
4.  **Attach the Payload**: Secure your payload by screwing it to your drone frame. Make sure it is properly attached.
5. **Attach the Battery**: Connect the battery to your drone via the clipping mechanism.
![Battery Attachment](https://github.com/user-attachments/assets/3147da73-95a3-4189-8f1e-322830f0c90f)

(Better to upgrade to GIF)

6.  **Check Center of Mass**: Use the Center of Mass (CoM) station to ensure that the center of mass is reasonably in the middle of the drone frame. You can do this by checking if all four scales show around the same weight. Adjust the battery position as necessary.

## Setup Instructions:

1.  **Place The Drone In the Testing Area**: Place the drone in the middle of the testing area (on the X) and connect the battery.
2. **Find The Machine ID**: Connect to the drone using ssh pi@10.0.0.## (where ## is the drone IP address. This can be found on a sticker placed on the drone). Run the bash script ./find_Machine_ID.sh to find the machine ID. It should look something like this (insert screenshot of machine ID).

(Crop screenshot)
![Machine_ID_cropped](https://github.com/user-attachments/assets/acac029e-604c-4911-9eca-52a7761c12be)

3. **Connect To The Portal and Add Your Drone**: Open a web browser and go to https://fly.droneleaf.io/login. Register and log in. Head to drone types and add your drone type. Insert your drone name, Choose the on-board computer type, and insert the inverse thrust to weight ratio. This can be calculated using the following formula: Inv_Thrust_Weight = 1 / (Total thrust from four motors / Total weight of drone with payload). Finally add a small description for your reference.

(Add screenshots)

4. **Add A New Drone Instance**: Go to the drone instances tab and press add drone in the top right. Insert the machine ID that you obtained two steps ago and press connect. Fill the necessary information and add your Drone Instance.

5. **Obtain The Software License**: After your drone is successfully added go to licenses and choose the yearly license. In the discount code slot write IROS2024 and checkout. You have now successfully obtained the product license. 

6.  **Activate the Flight Controller Software**: Connect to the drone through terminal using the following code: ssh pi@10.0.0.## (Where ## is your drone IP) Password: raspberry. Once connected run the following code:  *. start_HEAR_FC*.

7. **Turn on the Transmitter**: Turn on the Radio Frequency Transmitter for the kill-switch.

8.  **Activate the Mission Controller Software**: On a new terminal in your local machine run ./launch_mission_control.sh

## Operating Instructions:

1.  **Start the Mission Control system**: Press **"s"** and Enter. 
2.  **Idling the Motors**: Idle the motors by Pressing **"i"** and Enter.
3. **Test The Kill-switch**: Activate the kill-switch and confirm motors stop rotating. After confirming, turn off the kill-switch.
4.  **Take Off**: Instruct the drone to take off by pressing **"t"** and Enter. After takeoff, allow the drone to auto-tune. You can then guide it to follow a predefined trajectory.
5. **Activate Desired Trajectory**: After the Auto-tuning is complete run the desired trajectory by pressing **"o"** for a circle trajectory or **"8"** for a figure eight trajectory.
6. **Land & Disarm**: Press **""L""** and Enter to land and then **"d"** and Enter to Disarm.
##### Please note that all letter commands are in small letter format.
## Conclusion

Follow these guidelines to ensure a smooth assembly and setup process for your drone during the workshop. Good luck!

## Disclaimer
The software and procedures outlined in this guide are currently in alpha testing and may contain errors or bugs that could affect performance. Users are advised to proceed with caution and to thoroughly test all functionalities in a controlled environment before deployment.
By using this software, you acknowledge that DroneLeaf LLC does not assume any responsibility for any damages, losses, or injuries that may occur as a result of its use. This includes, but is not limited to, property damage, personal injury, or any other consequences arising from operational failures or unexpected behavior of the drone.
It is strongly recommended that users familiarize themselves with all safety protocols and operational guidelines prior to engaging with the drone. Users should also ensure that they have the necessary permissions and licenses required for flying drones in their respective areas.
DroneLeaf LLC reserves the right to update or modify the software and procedures at any time without prior notice. Continued use of this software signifies your acceptance of these terms.

