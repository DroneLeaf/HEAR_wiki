# HEAR-CLI

`hear-cli` is a management tool designed to help droneleafers in daily development and operational tasks, by gathering and executing all repeated HEAR commands in a single CLI. `hear-cli` leverages the capabilities of yakuake terminal.

# Design Philosophy

- Make the feeling and overhead of hear-cli close to (or better) than native use of command line.
- Document processes in shell scripts.

# How to setup
## Install `hear-cli`

Version: *1.3.1.0*

>> we can install `hear-cli` via `pip` as a python package
>>

```bash
sudo apt update

sudo apt install build-essential libdbus-glib-1-dev libgirepository1.0-dev -y
# pip install dbus-python

sudo apt install python3-pip -y

sudo apt-get install jq -y

cd ~/HEAR_CLI
pip install --user "dist/hear_cli-1.3.1.0-py3-none-any.whl[yakuake]" --force

#Add the hear-cli bin to $PATH
echo 'export PATH="/home/$USER/.local/bin:$PATH"' >> ~/.bashrc

source ~/.bashrc

hear-cli  --install-completion

# Open a new terminal and start
```

How to UnInstall
```bash

pip uninstall hear-cli

```
*** IMPORTANT: hear-cli folder must be placed in the home directory, i.e. `~/HEAR_CLI` ***

# Definitions
- hear-cli core: The python framework that constitutes hear-cli structure.
- hear-cli program: a shell script under `scripts/program` folder with entry point code in `main.sh`. The idea of programs is to provide dynamic expansion capability to hear-cli without the need to change hear-cli core.
- hear-cli functions: a set of shell scripts under `scripts/functions` folder. These functions get called by hear-cli programs. hear-cli functions help in code reusability.
- local machine: The machine from which hear-cli got run.
- remote target: a machine accessible over the network.
- fleet: a collection of remote targets. Remote targets in fleet are defined in HEAR_Configurations.
  
# Data flow
## Programs
The data used by hear-cli programs may come from two sources:
- The `parameters.json` file in the same directory of the program.
- Common `HEAR_Configurations` folder located at the home directory of the local machine.

## Cached data
hear-cli stores temporary operational files in `~/.hear-cli` folder in local machine and remote targets.

# Programs Structure
At the heart of hear-cli is the following program structure:
1) `fleet` command: this facilitates running programs on remote targets.
2) `local_machine` command: this facilitates two things:
   a) Quick preparation of custom terminal layouts for easy operation.
   b) Conveniently run shell scripts on the local machine from hear-cli calling path.

# How to  run
## Commnads

All commands can be viewed by running

```
hear-cli --help
```
and you can get command and subcommand help with 
``` bash
hear-cli <command> --help
hear-cli <command> <subcommand> --help

#EX:
hear-cli fleet --help
```

# Contribution Guidelines

## create program

you can find a program called hello_world , just make a copy of it and fit as you need
 - `main.sh` file is just used for starting program and extract runtime parameters used during the workflow
- all deep tasks should be written at specific sh file and stored in `functions` folder, this file name should reflect it's mission.
- Shell file should have only one mission, for example , if you need to clone and compile hear_fc, you should create file called `hear_fc_clone.sh` , and `hear_fc_compile.sh` inside functions folder. and inside the `main.sh` inside program called `hear_fc_clone_and_compile` , you should call these files respectively to run them.

- Nested shell scripting is not allowed in functions. Functions must have a call stack depth of one only