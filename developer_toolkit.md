# Developer Toolkit
**Document Scope**: The setup used for HEAR development.

The aim of this document is to document the developer tools that are used within the team. Please adhere to these tools to minimize *integration surprises*.

## Development for HEAR

- We use vscode with Ubuntu.
- Extensions of vscode:
  - 'Doxygen Documentation Generator'
  - ROS for debugging support.



## Source Tree Structure at Development Host
Having the same source tree structure for all developers is quite helpful: it helps avoiding surprises and automating workflows.

Please adhere to the following source tree structure:
(~: user home directory, it must correspond to $HOME environment variable)

~/HEAR_FC

~/HEAR_MC

~/HEAR_SITL

~/HEAR_configurations

~/Rpi_dep

~/PX4-Autopilot

**Important**: Make sure that all paths within code repos are not specific to your host PC, i.e. relative paths.

**Hint**: Use `getHomeDirectroy()` from `UnixEnvHelperFunc.hpp` within `HEAR_util` to avoid absolute links within the code

> When using VSCode make sure paths in .vscode folder are relative and use portable environment variables.



## Deployment Target