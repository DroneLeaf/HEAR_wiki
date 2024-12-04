# Developer Toolkit
**Document Scope**: The setup used for HEAR development.

The aim of this document is to document the developer tools that are used within the team. Please adhere to these tools to minimize *integration surprises*.

## Development tools for HEAR

- We use vscode with [Development Target](adding_dependencies.md#current-targets).
- Extensions of vscode:
  - 'Doxygen Documentation Generator'
  - ROS for debugging support.

## Nominal development workflow

1. Development happens on the [Development Target](adding_dependencies.md#current-targets).
2. You have modified HEAR_FC or HEAR_MC:
   1. Test compile.
   2. Run on the SITL environment (instructions on HEAR_SITL).
   3. All settings from: 
      1. HEAR_configurations, 
      2. HEAR_MC, 
      3. and [Configurations.cmake](adding_dependencies.md#structuring-configurationscmake) definitions 
      
      must fully define the target behaviour (except PX4 firmware and parameters, see PX4-Autopilot repo).
   4. Push your changes to the devel branches.
   5. Run Docker for the correct [Deployment Target](adding_dependencies.md#current-targets) on the Development Target.
   6. The Docker image, once build successfully, uploads the cross-compiled files to the remote file share.
   7. On the deployment target fetch the cross-compiled files from the remote file share.
   8. The target resolves it is behaviour from:
      1. HEAR_configurations stored at the target. Parameters are linked against the local target IP.
      2. HEAR_MC running on the operator machine (qualifies as a Development Target).
   9. PX4 firmware and parameters are uploaded to the correct Pixhawk target as instructed in the PX4-Autopilot repo.

## Source Tree Structure at Development and Deployment Host
Having the same source tree structure for all developers is quite helpful: it helps avoiding surprises and automating workflows.

Please adhere to the following source tree structure:
(~: user home directory, it must correspond to $HOME environment variable)

~/HEAR_FC

~/HEAR_MC

~/HEAR_SITL

~/HEAR_configurations

~/PX4-Autopilot

~/HEAR_wiki

~/HEAR_Docker

**Important**: Make sure that all paths within code repos are not specific to your host PC, i.e. relative paths.

**Hint**: Use `getHomeDirectroy()` from `UnixEnvHelperFunc.hpp` within `HEAR_util` to avoid absolute links within the code

> When using VSCode make sure paths in .vscode folder are relative and use portable environment variables.



## Deployment Target