# VS Code Workspace Setup notes:
Each development machine should have VS Code installed as the primary IDE for development.

## VS Code and Recommended Extensions
   ```bash
   code --install-extension ms-vscode.cpptools --force     # C/C++
   code --install-extension ms-vscode.cmake-tools --force   # CMake Tools 
   code --install-extension ms-python.python --force        # Python
   code --install-extension ms-python.vscode-pylance --force # Pylance
   code --install-extension mhutchie.git-graph --force      # GitGraph
   code --install-extension ms-azuretools.vscode-docker --force # Docker
   code --install-extension ms-iot.vscode-ros --force       # ROS Support 
   code --install-extension johnpapa.vscode-peacock --force # Peacock for color coding workspaces 
   ```
## Repository-specific VS Code Settings
each cloned repository contains a `.vscode` directory with recommended settings and launch configurations for that specific repo:
- 'launch.json' for debugging configurations
- 'settings.json' for workspace-specific settings
- etc.
Do not edit and commit changes to these files unless you intend to share modifications with all users of that repository.
<!-- Note -->
> Note: If configurations are not correct, please contact the repository maintainer for update.
