# Source management
**Document Scope**: The git repos used across HEAR and how to handle them.

## Guides
**Rules**:
- No forked repos.
- Only two branches on github (remote), `main` and `devel`.
- `main` is to be used for Applications Developers. All commits on `main` works. 
- The `Readme.md` in the `main` of the *base repo* must reference the commit references in the `main` of relevant repos that are compatible with the current repo. `HEAR_FC` by definition is used as the base repo.
- All development activities happen in `devel` branch.
- `HEAR_FC`, `HEAR_MC` and all of their submodules must have one owner.
- Local branchs are allowed, but must not be published to Github.
- All code bases must compile successfully before merging.

## Source dependency 

graph (visualize on GitHub or Mermaid):

```mermaid
flowchart TD
    A[HEAR_util] -->B(HEAR_mission)
    A -->C(HEAR_blocks)
    A -->D(HEAR_FC)
    A -->F(HEAR_interfaces)
    B -->D
    C -->D
    F -->D
    A -->E(HEAR_MC)
    B -->E
    C -->E
    F -->E
    G(HEAR_SITL)
    H(PX4-AutoPilot)
```

**Note** the source dependency graph provides a general rule on how code must be written and reused. e.g. write in `HEAR_util` first and reuse on other dependent modules. If not then move down the graph, i.e. `HEAR_mission`, `HEAR_blocks` or `HEAR_interfaces`, and so on.

**Converse Note** e.g. if you write a code in `HEAR_util` that gets used only in `HEAR_FC` and you do not see that in the future it would be used in any other repo rather than `HEAR_FC`, then that particular code must be removed from `HEAR_util` and be moved to `HEAR_FC`.

## Repos list
(Please copy paste 'Overview' section under each repo)

### HEAR_blocks
This repository contains all source files that inherit from the `Block` class, and all the coding infrastructure that supports Blocks coding paradigm like `System` and `Port` classes. This coding infrastructure is under `Blocks_core` folder.

### HEAR_mission
This repository contains all source files that inherits from the `MissionElement` class, and all the coding infrastructure that supports the Pipeline coding paradigm like `MissionPipeline` and `MissionScenario` classes. This coding infrastructure is under `Mission_core` folder.

### HEAR_util
This repository includes all functionalities that do not fit in the Blocks or Pipeline coding paradigms. Ideally, a functionality must be implemented here first to be encapsulated later by a Block or a MissionElement. For example, a code to write to a file must be written as a function in `HEAR_util` and then this function gets called within a `Block` or a `MissionElement`.

### HEAR_Interfaces
This repository contains all source files that inherits from the `InterfaceController` class and extends `InterfaceFactory` class. The role of this repo is to provide framing/deframing infrastructure of specific protocols and make them ready for consumption within Blocks coding paradigm.

### HEAR_FC
A code repo unifying `HEAR_blocks`, `HEAR_mission` and `HEAR_util` in a way to form a fully functional flight controller.

### [HEAR_MC](https://github.com/HazemElrefaei/HEAR_MC)
A code repo unifying `HEAR_blocks`, `HEAR_mission` and `HEAR_util` in a way to form a fully functional mission management software.

### [HEAR_SITL](https://github.com/MChehadeh/HEAR_SITL)

### [HEAR_ROS_bag_reader]()

### [PX4-AutoPilot](https://github.com/Mu99-M/PX4-Autopilot)
This is a re-published (not forked) repo of the original PX4-Autopilot. It also includes the settings file of PX4 for different variants of UAVs, and the relevant setup information. The main applications located in the `src/modules` directory.

### [HEAR_docker]()

### [HEAR_configurations](https://github.com/MChehadeh/HEAR_configurations)

### [DNN_system_ID](https://github.com/abdullaayyad96/DNN_system_ID)

