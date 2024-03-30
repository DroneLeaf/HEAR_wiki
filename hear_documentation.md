# HEAR Documentation

<!-- - COMMENT AA: General comments: I reviewed the first section. Overall it's very good. Please check my comments for your review. The main thing that lacks now is to have one full coherent example that guides the reader to write their own first block, system, executable, and run them. In other words we need full Hellow world example. There are some examples already under HEAR_SYSTEMS you can link them here. -->

## 1. HEAR_FC Essentials

### 1.1. Structure of a HEAR Code
<!-- - COMMENT AA: This is the structure of HEAR_FC only right? HEAR_Mission has different structure using elements and pipelines-->
<!-- - Brief overview of the block -> system -> optional system of systems -> executable structure. -->

The HEAR code follows a modular structure consisting of blocks, systems, optional system of systems, and executables:

1. Blocks: Blocks are the building unit of HEAR, representing individual components of a system. They can have synchronous (sync) and asynchronous (async) ports for communication.

2. Systems: Systems connect multiple blocks together to form a coherent functionality. Systems can also be nested to create a system of systems.

3. Executables: Executables are the final runnable components of the HEAR code. They are created using launch files. <!-- - COMMENT AA: executables are not created using launch file. You cna instead specify that each executable is a ros node that is compiled using catkin, and can be run using a launch file -->

### 1.2. HEAR Repositories
<!-- - COMMENT AA: Shall this be first? before th estructure? -->
<!-- - Description of HEAR_Blocks, HEAR_util, HEAR_Interfaces, and their functions. -->
<!-- NOTE: This sub-section is taken from source_management.md -->

#### HEAR_blocks
This repository contains all source files that inherit from the `Block` class, and all the coding infrastructure that supports Blocks coding paradigm like `System` and `Port` classes. This coding infrastructure is under `Blocks_core` folder.

#### HEAR_mission
This repository contains all source files that inherits from the `MissionElement` class, and all the coding infrastructure that supports the Pipeline coding paradigm like `MissionPipeline` and `MissionScenario` classes. This coding infrastructure is under `Mission_core` folder.

#### HEAR_util
This repository includes all functionalities that do not fit in the Blocks or Pipeline coding paradigms. Ideally, a functionality must be implemented here first to be encapsulated later by a Block or a MissionElement. For example, a code to write to a file must be written as a function in `HEAR_util` and then this function gets called within a `Block` or a `MissionElement`.

#### HEAR_Interfaces
This repository contains all source files that inherits from the `InterfaceController` class and extends `InterfaceFactory` class. The role of this repo is to provide framing/deframing infrastructure of specific protocols and make them ready for consumption within Blocks coding paradigm.
<!-- - COMMENT AA: Mention here the current interfaces that we support (ROS, MAVLINK, System Interface that is hear-specific, etc ...) -->

#### HEAR_FC
A code repo unifying `HEAR_blocks`, `HEAR_mission` and `HEAR_util` in a way to form a fully functional flight controller.

#### [HEAR_MC](https://github.com/HazemElrefaei/HEAR_MC)
A code repo unifying `HEAR_blocks`, `HEAR_mission` and `HEAR_util` in a way to form a fully functional mission management software.

#### [HEAR_SITL](https://github.com/MChehadeh/HEAR_SITL)

#### [HEAR_ROS_bag_reader]()

#### [PX4-AutoPilot](https://github.com/Mu99-M/PX4-Autopilot)
This is a re-published (not forked) repo of the original PX4-Autopilot. It also includes the settings file of PX4 for different variants of UAVs, and the relevant setup information. The main applications located in the `src/modules` directory.

#### [offboard_testing](https://github.com/Mu99-M/offboard_testing)
This repository installs MAVLink, MAVROS and offb packages.
1. MAVLink: A lightweight communication protocol designed for the exchange of information between unmanned systems and their ground control stations.

2. MAVROS: A ROS package that acts as a bridge between MAVLink-based autopilots and ROS.

3. offb: Enables external ROS nodes to send high-level commands to a MAVLink-enabled autopilot, allowing for advanced and customized control within the ROS ecosystem. The offb node has 3 main jobs:
    1. Switch to offboard mode and checks if it's disconnected to try switching again.
    2. Arm the vehicle and checks if it's disarmed to try arming again.
    3. Publishing values to SITL.

#### [HEAR_Docker](https://github.com/ahmed-hashim-pro/HEAR_Docker)
To cross-compile on your local machine and execute generated code on another one using docker Cross Compilation , To make a full installation of all prerequisites and dependencies needed to start your workspace in that target OS . In other words , this repo will install ros, cmake, vcpkg and other dependencies on the OS itself for fast dev setup

#### [HEAR_configurations](https://github.com/MChehadeh/HEAR_configurations)

#### [DNN_system_ID](https://github.com/abdullaayyad96/DNN_system_ID)




### 1.3. Creating a Block
<!-- - Walkthrough on creating a block with sync and async ports.
- Explanation of different port types and main methods within a block (process, processAsync, init, reset, etc.). -->

In the HEAR framework, blocks are the basic building units that process data. Blocks can have synchronous (sync) ports and asynchronous (async) ports for. This guide will walk you through the process of creating a block with both sync and async ports.
<!-- - COMMENT AA: explain here the Blocks have two main methods, process and processAsync. Where process is meant to read from input syncronous ports and write to output synchronous ports, similar for process Async -->
<!-- - COMMENT AA: Also here provide brief explanation on the different subfolder within HEAR_Blocks (HEAR_navigation, HEAR_math, etc ....). Highlight that if a new folder was created, then you would need to add it to cmakefiles -->


#### Step 1: Define the Block Class

First, create a new C++ header file for your block (e.g., `MyBlock.hpp`) and define the block class. In the block class, you will define the input and output ports, as well as the methods for processing data.
<!-- - COMMENT AA: you should highlight some main points here: - should inherit from Block class, all ports should be defined in the header file and should have a corresponding entry in the enums -->
<!-- - COMMENT AA: where should this header file be located? -->

<!-- - COMMENT AA: Is there anything that we need to highlight regarding the reset() method? -->


<!-- - COMMENT AA: What are the supported Input and Output types? If a custom type we need to create, how can it be done? (If this is part of a seperate documentation, then we at least need to mention it) -->

```cpp
#pragma once

namespace HEAR {

class MyBlock : public Block {

private:
    // Define input and output ports here
    InputPort<MyInputType>* _sync_input_port;
    OutputPort<MyOutputType>* _sync_output_port;

    AsyncInputPort<MyInputType>* _async_input_port;
    AsyncOutputPort<MyOutputType>* _async_output_port;

public:
    // Enumerate input and output port IDs
    enum IP { SYNC_INPUT_PORT_ID, ASYNC_INPUT_PORT_ID };
    enum OP { SYNC_OUTPUT_PORT_ID, ASYNC_OUTPUT_PORT_ID };

    // Constructor and Destructor
    MyBlock();
    ~MyBlock() {}

    void process();
    void processAsync() {}
    void reset() {}
    std::string getTypeDescription();

};

} // namespace HEAR
```

#### Step 2: Implement the Block Class

Next, implement the block class in a C++ source file (e.g., `MyBlock.cpp`). In the constructor, create the input and output ports for both sync and async data. Implement the process method to process sync data.

```cpp
#include "HEAR_path_to/MyBlock.hpp"

namespace HEAR {

MyBlock::MyBlock() : Block() {
    // Create input and output ports
    _sync_input_port = createInputPort<MyInputType>(IP::SYNC_INPUT_PORT_ID, "SYNC_INPUT_PORT_NAME");
    _sync_output_port = createOutputPort<MyOutputType>(OP::SYNC_OUTPUT_PORT_ID, "SYNC_OUTPUT_PORT_NAME");

    _async_input_port = createAsyncInputPort<MyInputType>(IP::ASYNC_INPUT_PORT_ID, "ASYNC_INPUT_PORT_NAME");
    _async_output_port = createAsyncOutputPort<MyOutputType>(OP::ASYNC_OUTPUT_PORT_ID, "ASYNC_OUTPUT_PORT_NAME");
}

void MyBlock::process() {
    // Read data from input port
    MyInputType sync_input_data;
    _sync_input_port->read(sync_input_data);

    // Perform all needed processing here, and store value in output variable

    // Write data to output port
    _sync_output_port->write(sync_output_data);
}

void MyBlock::processAsync() {
    if (_async_input_port->wasUpdated_AsyncIP()) {
        MyInputType async_input_data;
        _async_input_port->read_AsyncIP(async_input_data);
    }
}

void MyBlock::reset() {
    // Reset block state as needed

}

std::string MyBlock::getTypeDescription() {
    return "MyBlock";
}

} // namespace HEAR
```


<!-- - COMMENT AA: Can we provide here a standard example of a block that developers can refer to, for instance, the Multipy block as a reference for sync ports, and another block type for async ports -->

#### Step 3: Use the Block

<!-- - COMMENT AA: Since this is actually something that is done in a System, shall we instead move this to the system section? Here it would suffice to briefly summarize that blocks are connected in a system-->

Once you have created a block, you can use it in your system and connect its ports to other blocks' ports. Here's how you can do it in your system:

```cpp
auto my_block = new MyBlock();
this->addBlock(my_block, "MyBlock_Name");
```

To connect ports between blocks, use the connect method. For example, to connect the output port of Block1 to the input port of my_block:

```cpp
this->connect(Block1->getOutputPort<OutputType>(Block1::OP::OUTPUT_PORT_ID), my_block->getInputPort<InputType>(MyBlock::IP::INPUT_PORT_ID));
```

To connect the output port of my_block to the input port of Block2:

```cpp
this->connect(my_block->getOutputPort<OutputType>(MyBlock::OP::OUTPUT_PORT_ID), Block2->getInputPort<InputType>(Block2::IP::INPUT_PORT_ID));
```

Similary, to connect async ports between blocks, use the connectAsync method. For example, to connect the async output port of Block3 to the async input port of my_block:

```cpp
this->connectAsync(Block3->getAsyncOutputPort<AsyncOutputType>(Block3::OP::ASYNC_OUTPUT_PORT_ID), my_block->getAsyncInputPort<AsyncInputType>(MyBlock::IP::ASYNC_INPUT_PORT_ID));
```

This completes the process of using a block and connecting its ports in the HEAR framework. You can use similar steps to use other blocks and connect their ports as needed.

### 1.4. Creating a System
<!-- - How to connect multiple blocks through a system.
- Creating a system of systems. -->

In the HEAR framework, a system is a collection of connected blocks that work together to achieve a specific functionality. Here's a template for creating a system and connecting multiple blocks through it.

<!-- - COMMENT AA: First explain where are the systems located? and how are they organized in folders? How are these folders linked to the CMakeLists -->

#### Step 1: Define the System Class

First, create a new C++ header file for your system (e.g., `MySystem.hpp`) and define the system class. In the system class, you will define the input and output ports, as well as the blocks that constitute the system.

<!-- - COMMENT AA: do systems have input and output ports? -->

```cpp
#pragma once

#include "HEAR_core/System.hpp"

namespace HEAR {

class MySystem : public System {
public:
    MySystem(const std::string& sys_name);
    ~MySystem();
    std::string getTypeDescription() override;

private:
    void initBlocksLayout() override;
    void preProcessLogic() override;
    void postProcessLogic() override;
    void reset();
    void process() override;
    void processAsync() override;
};

}
```

#### Step 2: Implement the System Class

Next, implement the system class in a C++ source file (e.g., `MySystem.cpp`). In the constructor, define the blocks and connect their ports. You can also implement any pre-process, post-process, and process methods as needed.

```cpp
#include "HEAR_systems/MySystem.hpp"

namespace HEAR {

MySystem::MySystem(const std::string& sys_name) : System(sys_name) {}

MySystem::~MySystem() {}

void MySystem::reset() {};

std::string MySystem::getTypeDescription() {
    return "MySystem";
}

void MySystem::initBlocksLayout() {
    // Add the interfaces
    
    // Define blocks and their ports
    
    // Connect ports between blocks
    
    // Create async input and output ports

}

void MySystem::preProcessLogic() {}

void MySystem::postProcessLogic() {}

void MySystem::process() {
    // Implement system processing logic

    // Call process methods of individual blocks

}

void MySystem::processAsync() {
    // Implement async processing logic 

}

}
```

<!-- - COMMENT AA: I believe it's better to explain here how to connect blocks, since now we can highlight that they should be implemented in the initBlocksLayout method-->

<!-- - COMMENT AA: It's also important here to show an example of using the system interface-->



#### Creating a system of systems

<!-- - COMMENT AA: First, please explain why would we need a system for systems? for instance, explain that we might need to group different systems together like doing state estimation and control. In such a case, we need to run both systems and interchange information between them-->

In the system class, you will define the subsystems and their initialization.

```cpp
#pragma once

#include "HEAR_core/System.hpp"

namespace HEAR {

class MySystem : public System {

/* 
Code goes here
*/

private:
    // Declare your subsystems here
    MySubsystem1* subsystem1;

};

}
```

Next, implement the system class in a source file (e.g., `MySystem.cpp`). In the constructor, create and initialize the subsystems. Add the subsystems to the system using the addBlock method.

```cpp
void MySystem::initBlocksLayout() {
    // Create and initialize subsystems
    subsystem1 = new MySubsystem1();

    // Add the used interfaces to the subsystem (e.g., ros interface)
    subsystem1->addInterfaceFactory(itfc_fact_ros, "ros_interface");

    // init the subsytem
    subsystem1->init();

    // Add subsystems block to the system
    this->addBlock(subsystem1, "Subsystem1_Name");
}
```
<!-- - COMMENT AA: Here you should explain how we can interchange information between two systems using the system interface-->


### 1.5. Creating an Executable
<!-- - Steps to create an executable and a launch file. -->

#### Step 1: Create a sub-directory for the executable:

Create a directory at `/HEAR_FC/src/HEAR_FC/Flight_controller/HEAR_executables/scratch` (e.g., directory name will be `example`).

#### Step 2: Create a CPP File for the system:

Inside the `example` directory, create a `custom_executable.cpp` file with the system code.

<!-- - COMMENT AA: Explain here that this should be a ros node, and provide an example showing the main components of a ros node. Also show here that you actually use the systems -->


#### Step 3: Add a CMake File:

Inside the `example` directory, create a `CMakeLists.txt` file to add your `custom_executable.cpp` to it. You can use the following template as a guide:

```cmake
add_executable(custom_executable custom_executable.cpp)
add_dependencies(custom_executable ${catkin_EXPORTED_TARGETS} hear_msgs_generate_messages_cpp)
target_link_libraries(custom_executable
    Threads::Threads
    HEAR_ROS
    HEAR_util
    hear
    hear_mission
    ${catkin_LIBRARIES}
)

if(ONNX_DEPS)
target_link_libraries(custom_executable
    ${libonnxruntime_path}
)
endif()
```

#### Step 4: Add your sub-directory to the HEAR_executables:

Open `/HEAR_executables/scratch/CMakeLists.txt`.
Add the following line at the end of the file:

```cmake
add_subdirectory(example)
```

#### Step 5: Create a Launch File:

Create a launch file at `/HEAR_FC/src/HEAR_FC/Flight_controller/launch` (e.g., `custom_executable.launch`). Add the content to run your excutable, you can use the following template as a guide:

```xml
<launch>
  <arg name="DRONE_NAME"/> 

  <remap from="/tf" to="/$(arg DRONE_NAME)/nodelet_manager/tf"/>

  <node name="custom_executable" pkg="flight_controller" type="custom_executable" output="screen"/>

</launch>
```

### 1.6. HEAR System Interface
- Adding a custom message type to HEAR.

## 2. HEAR Interfaces

### 2.1. ROS Interfaces
- Creating a ROS subscriber, publisher, server, and client.
- Adding a custom message or service type.

### 2.2. Mavlink Interface
- Creating a MAVLink subscriber, publisher, server, and client.
- Adding a custom MAVLink message to HEAR.

## 3. HEAR Configurations / Parameters
<!-- Comment AA: Add here a brief description on the purpose of the parameters in HEAR Configurations, they are meant to include all parameters that can alter the drone's performance or behaviour without the need to recompile  -->

<!-- Comment AA: You also need to describe here that all parameters are stored in Json files, and that the files are organized in a structured manner   -->
### 3.1. HEAR Configurations Structure
<!-- - Overview of UAV_types, instances, Systems, etc. -->

HEAR configurations are structured around defining various aspects of the UAV setup, including UAV_types, instances, systems, etc. This structured approach helps in organizing and managing UAV systems efficiently.

- UAV_types: Defines the different types of UAVs in the system, such as quadcopters, hexacopters, etc.
  <!-- Comment AA: Highlight the structure of UAV_instances (each instant has it's own folder, and in that folder is a general.json file that containt the parameters of that instance ....    -->

    - Examples: Big_Hexa_DCDC, Quad, Octa

- UAV_instances: Defines the instances of UAV type, specifying unique identifiers for each UAV instance.

    - Example: UAV1, Sim_Big_Hexa_DCDC

- Systems: Used to specify PX4 control and communication settings.
<!-- Comment AA: Add more example on the systems here. Mainly parameters related to optitrack, Simulator, Propulsions, etc ...  -->



### 3.2. Reading Parameters
<!-- - How to read a parameter from HEAR Configurations. -->
<!-- Comment AA: This needs much more details, explain that HEAR contains a parameters interface that is tasked with reading parameters from the Configurations directory and providing them whenever needed. Highlight that we usually use this parameter interface in the system level  -->
You can read parameters from configurations, such as system settings, instances, types, etc. 

<!-- - Example of reading a parameter from Systems, instances, types, etc. -->
Here's an example of how to read a parameter from HEAR Configurations:

<!-- Comment AA: Where is this code usually placed?? this need to be highlighted -->

<!-- Comment AA: the code below assumes the user knows what `config_ctrl` is? where is it declared and what is it? -->

```cpp
UAVConfigController* config_ctrl = dynamic_cast<UAVConfigController*>(intfc_fact_config->getController());

// Read parameters from the configuration file
auto use_mavros = config_ctrl->getValueFromFile<bool>(config_ctrl->getSystemsSettingsFilePath(), "px4_over_mavros");

    if (use_mavros) {
        /* 
        code goes here
        */
    }
```

In this example, we use `UAVConfigController` to read a boolean parameter `px4_over_mavros` from the system settings file. Based on the value read, the code initializes the MAVROS interface if `px4_over_mavros` is set to true.

<!-- Comment AA:You can refer the user to a coupel of systems here where reading from parameters is used, such that they can refer back into it. -->

<!-- - Supported variable types. -->
<!-- Comment AA: Where are the supported variable types?. -->

### 3.3. Adding a New Parameter File
- Adding a new parameter file related to a new system.
- Reading from a new file.
<!-- Comment AA: Where is this? -->

## 4. HEAR Missions <!-- TODO: MK: this section still needs more work on it -->

### 4.1. Anatomy of a HEAR Mission
<!-- - Explanation of mission elements, mission pipelines, etc. -->
HEAR Missions are composed of **mission elements** and **pipelines**.

- Mission element: Represents a specific task or action.

- Pipeline: A sequence of mission elements that form a complete mission scenario.

- Graph Display (Optional): Display for graphs, created using GraphDisplayDriverG (if GUI_Display is defined).

Missions are executed sequentially, with each mission element completing before the next one begins.


### 4.2. Creating a Mission Element
<!-- - How to create a mission element. -->
To create a mission element, you can use the `MissionScenarioG` class.

### 4.3. Creating a Mission Pipeline
<!-- - How to create a mission pipeline. -->
A mission pipeline is a sequence of mission elements that define a complete mission scenario. Pipelines are executed sequentially, with each mission element completing before the next one begins. To create a mission pipeline, you can use the `MissionPipelineG` class. This class allows you to add mission elements to the pipeline and define the order in which they should be executed.

### 4.4. Demonstration
<!-- - Complete demonstration on creating and running a mission scenario. -->

## 5. Testing and Debugging
<!--Comment AA: First provide a quick overview of the testing types available -->

### 5.1. Testing in Gazebo SITL
<!--Comment AA: YOu can first refer the reader to check the github repo of PX4-Autopilot for guidelines on installing the PX4 dependencies -->
- How to run and test the system in Gazebo SITL.
  <!--Comment AA: This is very important, please add ASAP -->
  

### 5.2. Testing in HEAR SITL
<!-- - How to run and test the system in HEAR SITL for both quadrotor and hexarotor. -->
<!-- - Parameters that should be changed. -->
<!--Comment AA: First explain what is HEAR SITL, and how is it different from gazebo -->

#### Setting Up Quadrotor

1. Build px4 in sitl (repo: PX4-AutoPilot, dev branch):

<!--Comment AA: is the hear_quad updates pushed to github? -->
    ```bash
    cd ~/PX4-Autopilot
    make px4_sitl_default hear_quad
    ```
1. Update Configuration:

    Change the `"default_uav_instance_name"` parameter in `UAV_instances` in the configurations to a valid quadrotor vehicle.

#### Setting Up Hexarotor

1. Build px4 in sitl (repo: PX4-AutoPilot, dev branch)

    ```bash
    cd ~/PX4-Autopilot
    make px4_sitl_default hear_hexa
    ```

2. Update Configuration:

    Change the `"default_uav_instance_name"` parameter in `UAV_instances` in the configurations to a valid hexarotor vehicle.
<!--Comment AA: provide an example of a hexarotor vehicle-->

#### Testing Process (for both Quadrotor or Hexarotor)

<!--Comment AA: This is optional, only if running through mavros. We can also run it through mavlink. In our next call I can show you how to run through mavlink-->
1. Launch `px4.launch` node (repo: offboard_testing)

    ```bash
    roslaunch mavros px4.launch fcu_url:=udp://:14540@14555
    ```

2. Launch `PX4_SITL.launch` (repo: HEAR_FC)

    ```bash
    roslaunch flight_controller PX4_SITL.launch DRONE_NAME:=UAV1
    ```

3. Launch `px4_flight_controller.launch` (repo: HEAR_FC)

    ```bash
    roslaunch flight_controller px4_flight_controller.launch DRONE_NAME:=UAV1
    ```

4. Launch `starting.launch` (repo: offboard_testing)

    This is used for arming the vehicle.

    ```bash
    roslaunch offb starting.launch
    ```

5. Launch `mission_scenario` (repo: HEAR_MC)

    ```bash
    roslaunch hear_mc_example mission_scenario.launch
    ```

- You can echo the position from `vehicle_local_pos` topic

    ```bash
    rostopic echo /mavros/vehicle_local_position/vehicle_local_pos
    ```

    <!--Comment AA: Also guide the user that they can check and debug through QGround Control-->

### 5.3. Debugging ROS Topics/Services
<!-- - How to debug and check ROS topics/services. -->

To debug and check ROS topics/services, you can use the `TRACE_SYNC_OUT` macro. This macro is used to trace the output of a ROS topic subscriber.

    <!--Comment AA: The below example is not clear, where would it usually be placed? what is sub_topic and of what type is it?-->
```cpp
// Assuming `sub_topic` is your subscriber and `MsgType` is the message type
auto output_port = sub_topic->getOutputPort<MsgType>(ROSUnit_Subscriber<MsgType>::OP::OUTPUT);

TRACE_SYNC_OUT(output_port, MsgType, itfc_factory, "trace_name")
```

Replace `sub_topic` with your actual subscriber pointer, `MsgType` with your message type, `itfc_factory` with your interface factory, and `"trace_name"` with a descriptive name for the trace. This macro will help you monitor the output of your ROS topic subscriber for debugging purposes using this command:

```bash
rostopic echo trace_name
```

### 5.4. Debugging MAVLink Messages
<!-- - How to debug and check MAVLink messages using Wireshark. -->
#### Step 1: Generate MAVLink Messages using mavgen

1. Generate the Lua script with mavgen:

    - Use [mavgen](https://mavlink.io/en/getting_started/generate_libraries.html) to generate the Lua script for your custom MAVLink messages from the XML description of your messages (found at `/PX4-Autopilot/src/modules/mavlink/mavlink/message_definitions/v1.0/common.xml`).

2. Install Wireshark:
    <!--Comment AA: Can you add pictures for this process? -->
    
    - Download and install [Wireshark](https://www.wireshark.org/).

4. Locate the Wireshark plugins directory:

    - The plugins directory can be found at `/usr/lib/x86_64-linux-gnu/wireshark/plugins` for linux users. Please note that this path could be different for different devices.

5. Place your Lua script in the Wireshark plugins directory.

    - Copy or move the Lua script generated by mavgen into the new directory.

> Note: Repeat this step only if there are any MAVLink messages added, removed, or modified.

#### Step 2: Debug and check MAVLink messages using Wireshark

1. Capture MAVLink Messages:

    - Start Wireshark in super user mode: `sudo wireshark`.
    - Select the `Loopback: lo` interface (or any interface used for loopback communication).
    - Click on the "Start" button to begin capturing packets.

2. Filter MAVLink Messages:

    - Enter `mavlink_proto && not icmp` in the filter field to display only MAVLink messages 

3. Analyze MAVLink Messages:

    - Wireshark will display captured MAVLink messages. You can expand each message to view its details, including message ID, payload, and decoded fields.

4. Save Capture:
    
    - If you need to save the capture for later analysis, go to `"File" > "Save As"`.
