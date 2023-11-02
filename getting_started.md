# HEAR Development Getting Started Guide

## Before you code!

 1. Make sure you read all the relevant development guidelines
 2. Check existing code base for the functionality you want.
 3. Make sure you have discussed the proposed changes with the respective repo manager.

## After you code

 1. Review your changes. Make sure you adhered to the [coding guideline](coding_guideline.md).
 2. Document your changes. Add any new dependencies as in the [guideline](adding_dependencies.md). 

## Source Tree Structure at Host
Having the same source tree structure for all developers is quite helpful: it helps avoiding surprises and automating workflows.

Please adhere to the following source tree structure:
(~: user home directory)

~/HEAR_FC

~/HEAR_MC

~/HEAR_SITL

~/HEAR_configurations

~/Rpi_dep

*AA: please add PX4 autopilot*

**Important**: Make sure that all paths within code repos are not specific to your host PC, i.e. relative paths.

**Hint**: Use `getHomeDirectroy()` from UnixEnvHelperFunc.hpp within HEAR_util to avoid absolute links within the code

> When using VSCode make sure paths in .vscode folder are relative

## Repositories structuring



## Software Developer Types
### Architecture Developers
Responsibilities:
 1. Develop core architectural elements
 2. Develop communication interfaces

Clients served:
 1. Algorithms Developers
 2. Application Developers

Current Developers:
1. Mohamad Chehadeh
2. Ahmed Hashem

### Algorithms Developers



### Application Developers




