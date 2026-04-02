## Sim UI
* Remove OSD targetting crosshairs only
* Sim computer = Betaflight + Gazebo physics engine + Ground station application 
    * connected to TX16s via the gunner application 
    * Connected to LeafFC computer via two ports (add discovery to sim UI) with baud (narrow down to ttyUSB*)
	    * MSP port (specify baud rate: 230400)
	    * ELRS RX port (specify baud rate: 425000)
	* Simulation settings: 
		* World selection (chase fpv park, chase fpv (without terrain))
		* Physics selection (leaf-sim means simulink) and gazebo-sim
		* Drone selection - thaqib-1 drone = rocket_drone (betaflight_vehicle), iris drone
		* We need 6 scripts:
			* iris + chase fpv (without terrain) + gazebo sim
			* iris + chase fpv park+ gazebo sim
			*  thaqib-1 + chase fpv (without terrain)+ gazebo sim
			*  thaqib-1 + chase fpv park+ gazebo sim
			*  thaqib-1 + chase fpv (without terrain)+ leaf-sim
			*  thaqib-1 + chase fpv park+ leaf-sim
	* Visualization settings:
		* Chase cam ON/OFF
		* Gazebo UI ON/OFF
		* OSD ON/OFF